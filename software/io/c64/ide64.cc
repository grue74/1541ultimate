/*
 * ide64.cc
 *
 * IDE64 ATA device emulation. The FPGA (ide64_ata.vhd) handles the C64 bus,
 * the task file, BSY/DRQ and the 512-byte sector buffer; this driver executes
 * the ATA commands against a disk image file (.hdd / .cfa, as used by VICE).
 *
 * Behaviour follows VICE (src/core/ata.c, src/c64/cart/ide64.c).
 */

#include <stdio.h>
#include <string.h>
#include "ide64.h"
#include "itu.h"
#include "filemanager.h"
#include "init_function.h"

#define CFG_IDE64_IMAGE  0x01
#define CFG_IDE64_MOUNT  0x02

static struct t_cfg_definition ide64_config[] = {
    { CFG_IDE64_MOUNT,  CFG_TYPE_ENUM,   "Mount Image at Startup", "%s", en_dis, 0,   1, 0 },
    { CFG_IDE64_IMAGE,  CFG_TYPE_STRING, "Drive 0 Image",          "%s", NULL,   0, 127, (long)"" },
    { CFG_TYPE_END,     CFG_TYPE_END,    "",                       "",   NULL,   0,   0, 0 } };

static Ide64 *ide64_instance = NULL;

Ide64 *Ide64 :: getIde64()
{
    return ide64_instance;
}

InitFunction ide64_init("IDE64", [](void *_obj, void *_param) {
    if (getFpgaCapabilities() & CAPAB_IDE64) {
        ide64_instance = new Ide64();
        ide64_instance->init();
    }
}, NULL, NULL, 99);

/*********************************************************************/
/* Geometry                                                          */
/*********************************************************************/
static uint16_t be16(const uint8_t *p)
{
    return ((uint16_t)p[0] << 8) | p[1];
}

static uint32_t be32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
}

// detect_ide64_image() and the CHS calculation of ata_poweron() in VICE
void Ide64Drive :: detect_geometry(void)
{
    uint8_t header[24];
    uint32_t transferred = 0;

    size = file->get_size() >> 9;
    cylinders = heads = sectors = 0;

    file->seek(0);
    if ((file->read(header, 24, &transferred) != FR_OK) || (transferred < 24)) {
        memset(header, 0, sizeof(header));
    }

    if (memcmp(header, "C64-IDE V", 9) == 0) { // old filesystem, always CHS
        cylinders = be16(&header[0x10]) + 1;
        heads = (header[0x12] & 0x0f) + 1;
        sectors = header[0x13];
        size = (uint32_t)cylinders * heads * sectors;
    } else if (memcmp(header + 8, "C64 CFS V", 9) == 0) {
        if (header[0x04] & 0x40) { // LBA
            size = be32(&header[0x04]) & 0x0fffffff;
        } else { // CHS
            cylinders = be16(&header[0x05]) + 1;
            heads = (header[0x04] & 0x0f) + 1;
            sectors = header[0x07];
            size = (uint32_t)cylinders * heads * sectors;
        }
    }

    if ((sectors < 1) || (sectors > 63) || (cylinders > 65535) ||
        ((uint32_t)sectors * heads * cylinders > 16514064)) {
        uint32_t sz = (size > 16514064) ? 16514064 : size;
        uint32_t h = 1, s = 1, i = 63, c = sz;
        while ((i > 1) && (c > 1)) {
            if ((c % i) == 0) {
                if (s * i <= 63) {
                    s *= i; c /= i;
                    continue;
                }
                if (h * i <= 16) {
                    h *= i; c /= i;
                    continue;
                }
            }
            i--;
        }
        for (;;) {
            if (sz <= 1032192) {
                if (c <= 1024) {
                    break;
                }
            } else {
                if ((h < 5) && (c < 65536)) {
                    break;
                }
                if ((h < 9) && (c < 32768)) {
                    break;
                }
                if (c < 16384) {
                    break;
                }
            }
            if ((s == 63) && (h < 16)) {
                h++;
            }
            if (s < 63) {
                s++;
            }
            c = sz / (h * s);
        }
        cylinders = c;
        heads = h;
        sectors = s;
    }
    reset_translation();
}

/*********************************************************************/
/* Driver                                                            */
/*********************************************************************/
Ide64 :: Ide64()
{
    regs = (volatile ide64_regs_t *)IDE64_BASE;
    buffer = (volatile uint8_t *)(IDE64_BASE + IDE64_BUFFER_OFFSET);
    taskHandle = NULL;
    present_mask = 0;
    available = true;
    mutex = xSemaphoreCreateMutex();

    register_store(0x49444536, "IDE64 Settings", ide64_config);
}

void Ide64 :: init(void)
{
    regs->present = 0;
    regs->events = IDE64_EV_CMD | IDE64_EV_BUF | IDE64_EV_RESET;
    install_high_irq(ITU_IRQHIGH_IDE64, Ide64 :: irq, this);
    xTaskCreate(Ide64 :: task, "IDE64", configMINIMAL_STACK_SIZE, this, PRIO_DRIVER, &taskHandle);
}

uint8_t Ide64 :: irq(void *context)
{
    Ide64 *ide = (Ide64 *)context;
    BaseType_t woken = pdFALSE;

    ide->regs->present = ide->present_mask; // IRQ off until the task has handled the events
    vTaskNotifyGiveFromISR(ide->taskHandle, &woken);
    return (uint8_t)woken;
}

void Ide64 :: write_present(void)
{
    regs->present = present_mask | IDE64_PRESENT_IRQ;
}

void Ide64 :: task(void *a)
{
    ((Ide64 *)a)->run();
}

void Ide64 :: run(void)
{
    bool automount = (cfg->get_value(CFG_IDE64_MOUNT) != 0) && (cfg->get_string(CFG_IDE64_IMAGE)[0] != 0);
    int tries = 0;

    for (;;) {
        // Mount the startup image once the medium shows up (USB takes a while)
        if (automount && !is_mounted() && (tries < 60)) {
            tries++;
            if (mount(cfg->get_string(CFG_IDE64_IMAGE), NULL) == FR_OK) {
                automount = false;
            }
        }

        write_present();
        ulTaskNotifyTake(pdTRUE, automount ? 1000 : portMAX_DELAY);

        uint8_t ev = regs->events;
        if (ev & IDE64_EV_RESET) {
            regs->events = IDE64_EV_RESET;
            drive.reset_translation();
        }
        if (ev & IDE64_EV_BUF) { // stray, nothing waits for it here
            regs->events = IDE64_EV_BUF;
        }
        if (ev & IDE64_EV_CMD) {
            regs->events = IDE64_EV_CMD;
            xSemaphoreTake(mutex, portMAX_DELAY);
            execute(regs->command);
            xSemaphoreGive(mutex);
        }
    }
}

// Wait until one of the events in mask (or a reset) is pending. Returns the events.
// A pending reset is left for the main loop to acknowledge.
uint8_t Ide64 :: wait_event(uint8_t mask)
{
    mask |= IDE64_EV_RESET;
    for (;;) {
        uint8_t ev = regs->events;
        if (ev & mask) {
            return ev;
        }
        write_present();
        ulTaskNotifyTake(pdTRUE, 1000);
    }
}

void Ide64 :: copy_to_fpga(const uint8_t *src)
{
    for (int i = 0; i < 512; i++) {
        buffer[i] = src[i];
    }
}

void Ide64 :: copy_from_fpga(uint8_t *dest)
{
    for (int i = 0; i < 512; i++) {
        dest[i] = buffer[i];
    }
}

// Finish a command without a data phase (drops BSY)
void Ide64 :: complete(uint8_t error)
{
    regs->error = error;
    regs->status = ATA_STAT_DRDY | ((error & 0xFE) ? ATA_STAT_ERR : 0);
    regs->control = 0;
}

// Sector address from the task file; false if out of range
bool Ide64 :: get_lba(uint32_t &lba)
{
    uint8_t sel = regs->select;
    if (sel & 0x40) {
        lba = ((uint32_t)(sel & 0x0F) << 24) | ((uint32_t)regs->hcyl << 16) | ((uint32_t)regs->lcyl << 8) | regs->sector;
    } else {
        uint32_t cyl = ((uint32_t)regs->hcyl << 8) | regs->lcyl;
        uint32_t head = sel & 0x0F;
        uint32_t sec = regs->sector;
        if ((sec == 0) || (sec > drive.cur_sectors) || (head >= drive.cur_heads) || (cyl >= drive.cur_cylinders)) {
            return false;
        }
        lba = (cyl * drive.cur_heads + head) * drive.cur_sectors + sec - 1;
    }
    return lba < drive.size;
}

// Report the failing sector address in the task file (ata_set_command_block)
void Ide64 :: set_command_block(uint32_t lba)
{
    uint8_t sel = regs->select;
    if (sel & 0x40) {
        regs->select = (sel & 0xF0) | ((lba >> 24) & 0x0F);
        regs->hcyl = (uint8_t)(lba >> 16);
        regs->lcyl = (uint8_t)(lba >> 8);
        regs->sector = (uint8_t)lba;
    } else if (drive.cur_sectors && drive.cur_heads) {
        uint32_t cyl = lba / drive.cur_sectors / drive.cur_heads;
        regs->sector = (uint8_t)(lba % drive.cur_sectors + 1);
        regs->select = (sel & 0xF0) | ((lba / drive.cur_sectors) % drive.cur_heads);
        regs->lcyl = (uint8_t)cyl;
        regs->hcyl = (uint8_t)(cyl >> 8);
    }
}

void Ide64 :: execute(uint8_t command)
{
    if (!drive.file) {
        complete(ATA_ERR_ABRT);
        return;
    }
    switch (command) {
    case 0x20: case 0x21: // READ SECTORS
        cmd_read();
        break;
    case 0x30: case 0x31: // WRITE SECTORS
        cmd_write();
        break;
    case 0x40: case 0x41: // READ VERIFY SECTORS
    case 0x70:            // SEEK
        cmd_verify();
        break;
    case 0x90: // EXECUTE DEVICE DIAGNOSTIC
        regs->nsect = 1;
        regs->sector = 1;
        regs->lcyl = 0;
        regs->hcyl = 0;
        regs->select = 0;
        complete(0x01); // device 0 passed
        break;
    case 0x91: // INITIALIZE DEVICE PARAMETERS
        cmd_init_params();
        break;
    case 0xEC: // IDENTIFY DEVICE
        cmd_identify();
        break;
    case 0xEF: // SET FEATURES
        cmd_set_features();
        break;
    case 0xE4: // READ BUFFER: the buffer still holds the last sector
        regs->error = 0;
        regs->status = ATA_STAT_DRDY;
        regs->control = IDE64_CTRL_ARM;
        break;
    case 0xE8: // WRITE BUFFER
        regs->error = 0;
        regs->status = ATA_STAT_DRDY;
        regs->control = IDE64_CTRL_ARM | IDE64_CTRL_DATA_OUT;
        if (wait_event(IDE64_EV_BUF) & IDE64_EV_RESET) {
            return;
        }
        regs->events = IDE64_EV_BUF;
        complete(0);
        break;
    case 0x98: case 0xE5: // CHECK POWER MODE: active
        regs->nsect = 0xFF;
        complete(0);
        break;
    case 0xE7: // FLUSH CACHE
        complete((drive.file->sync() == FR_OK) ? 0 : (ATA_ERR_UNC | ATA_ERR_ABRT));
        break;
    case 0x94: case 0x95: case 0x96: case 0x97: case 0x99: // power management: accepted
    case 0xE0: case 0xE1: case 0xE2: case 0xE3: case 0xE6:
        complete(0);
        break;
    default: // NOP (0x00) and everything else
        printf("IDE64: unsupported command %02x\n", command);
        complete(ATA_ERR_ABRT);
        break;
    }
}

void Ide64 :: cmd_read(void)
{
    uint32_t lba;
    int count = regs->nsect ? regs->nsect : 256;

    if (!get_lba(lba)) {
        complete(ATA_ERR_IDNF);
        return;
    }
    if (drive.file->seek(lba << 9) != FR_OK) {
        complete(ATA_ERR_IDNF);
        return;
    }
    for (int k = 0; k < count; k++, lba++) {
        uint32_t transferred = 0;
        if (lba >= drive.size) {
            set_command_block(lba);
            complete(ATA_ERR_IDNF);
            return;
        }
        if (drive.file->read(sector_data, 512, &transferred) != FR_OK) {
            set_command_block(lba);
            complete(ATA_ERR_UNC | ATA_ERR_ABRT);
            return;
        }
        if (transferred < 512) { // beyond the end of the file: zeros, as VICE does
            memset(sector_data + transferred, 0, 512 - transferred);
        }
        copy_to_fpga(sector_data);
        regs->error = 0;
        regs->status = ATA_STAT_DRDY;
        if (k < count - 1) {
            regs->control = IDE64_CTRL_ARM | IDE64_CTRL_MORE;
            if (wait_event(IDE64_EV_BUF) & IDE64_EV_RESET) {
                return;
            }
            regs->events = IDE64_EV_BUF;
        } else {
            regs->control = IDE64_CTRL_ARM;
        }
    }
}

void Ide64 :: cmd_write(void)
{
    uint32_t lba;
    int count = regs->nsect ? regs->nsect : 256;

    if (!get_lba(lba)) {
        complete(ATA_ERR_IDNF);
        return;
    }
    if (drive.readonly) {
        complete(ATA_ERR_WP | ATA_ERR_ABRT);
        return;
    }
    if (drive.file->seek(lba << 9) != FR_OK) {
        complete(ATA_ERR_IDNF);
        return;
    }
    for (int k = 0; k < count; k++, lba++) {
        uint32_t transferred = 0;
        if (lba >= drive.size) {
            set_command_block(lba);
            complete(ATA_ERR_IDNF);
            return;
        }
        regs->error = 0;
        regs->status = ATA_STAT_DRDY;
        regs->control = IDE64_CTRL_ARM | IDE64_CTRL_DATA_OUT;
        if (wait_event(IDE64_EV_BUF) & IDE64_EV_RESET) {
            drive.file->sync();
            return;
        }
        regs->events = IDE64_EV_BUF;
        copy_from_fpga(sector_data);
        if ((drive.file->write(sector_data, 512, &transferred) != FR_OK) || (transferred != 512)) {
            set_command_block(lba);
            complete(ATA_ERR_UNC | ATA_ERR_ABRT);
            return;
        }
    }
    complete((drive.file->sync() == FR_OK) ? 0 : (ATA_ERR_UNC | ATA_ERR_ABRT));
}

void Ide64 :: cmd_verify(void)
{
    uint32_t lba;
    if (!get_lba(lba)) {
        complete(ATA_ERR_IDNF);
        return;
    }
    if (regs->command != 0x70) {
        uint32_t count = regs->nsect ? regs->nsect : 256;
        if (lba + count > drive.size) {
            set_command_block(drive.size);
            complete(ATA_ERR_IDNF);
            return;
        }
    }
    complete(0);
}

void Ide64 :: cmd_init_params(void)
{
    drive.cur_heads = (regs->select & 0x0F) + 1;
    drive.cur_sectors = regs->nsect;
    if ((drive.cur_sectors < 1) || (drive.cur_sectors > 63)) {
        drive.cur_cylinders = 0;
    } else {
        uint32_t sz = (drive.size > 16514064) ? 16514064 : drive.size;
        sz /= drive.cur_heads * drive.cur_sectors;
        drive.cur_cylinders = (sz > 65535) ? 65535 : sz;
    }
    if (drive.cur_cylinders == 0) {
        drive.cur_heads = 0;
        drive.cur_sectors = 0;
        complete(ATA_ERR_ABRT);
        return;
    }
    complete(0);
}

void Ide64 :: cmd_set_features(void)
{
    switch (regs->features) {
    case 0x03: // SET TRANSFER MODE: PIO default / PIO with flow control only
        if ((regs->nsect > 1) && (regs->nsect != 8)) {
            complete(ATA_ERR_ABRT);
            return;
        }
        break;
    case 0x02: case 0x33: case 0x55: case 0x82: case 0x99: case 0xAA:
        break;
    default:
        complete(ATA_ERR_ABRT);
        return;
    }
    complete(0);
}

// ATA strings are stored with the bytes of each word swapped
static void ident_string(uint8_t *b, const char *s, int n)
{
    for (int i = 0; i < n; i += 2) {
        b[i | 1] = *s ? *s++ : ' ';
        b[i] = *s ? *s++ : ' ';
    }
}

void Ide64 :: cmd_identify(void)
{
    uint8_t *r = sector_data;
    memset(r, 0, 512);

#define PUTW(a, v) { r[(a) * 2] = (uint8_t)(v); r[(a) * 2 + 1] = (uint8_t)((v) >> 8); }
#define SETB(a, b) { r[(a) * 2 + (b) / 8] |= (1 << ((b) & 7)); }

    PUTW(0, drive.cf ? 0x848a : 0x0040);
    PUTW(1, drive.cylinders);
    PUTW(3, drive.heads);
    if (!drive.cf) {
        PUTW(4, 512 * drive.sectors);
        PUTW(5, 512);
    }
    PUTW(6, drive.sectors);
    if (drive.cf) {
        PUTW(7, drive.size >> 16);
        PUTW(8, drive.size);
    }
    ident_string(r + 20, "1541ULTIMATE", 20);   // serial number
    PUTW(21, 1);                                 // buffer size in sectors
    ident_string(r + 46, "1.0", 8);              // firmware revision
    ident_string(r + 54, drive.cf ? "ULTIMATE IDE64 CFA" : "ULTIMATE IDE64 HDD", 40);
    SETB(49, 13); // standard timers
    SETB(49, 9);  // LBA
    if (drive.cur_sectors) {
        SETB(53, 0);
        PUTW(54, drive.cur_cylinders);
        PUTW(55, drive.cur_heads);
        PUTW(56, drive.cur_sectors);
        uint32_t i = (uint32_t)drive.cur_cylinders * drive.cur_heads * drive.cur_sectors;
        if (i > drive.size) {
            i = drive.size;
        }
        PUTW(57, i);
        PUTW(58, i >> 16);
    }
    PUTW(60, drive.size);
    PUTW(61, drive.size >> 16);
    SETB(82, 3);  // power management
    SETB(82, 5);  // write cache
    SETB(82, 6);  // look-ahead
    SETB(82, 12); // write buffer
    SETB(82, 13); // read buffer
    SETB(83, 12); // flush cache
    SETB(83, 14);
    SETB(84, 14);
    SETB(85, 3);
    SETB(85, 12);
    SETB(85, 13);
    SETB(86, 12);
    SETB(87, 14);
    PUTW(255, 0xa5);
    for (int i = 0; i < 511; i++) {
        r[511] -= r[i];
    }
#undef PUTW
#undef SETB

    copy_to_fpga(r);
    regs->error = 0;
    regs->status = ATA_STAT_DRDY;
    regs->control = IDE64_CTRL_ARM;
}

/*********************************************************************/
/* Mounting                                                          */
/*********************************************************************/
// filename may be NULL, then path is the full path name
FRESULT Ide64 :: mount(const char *path, const char *filename)
{
    FileManager *fm = FileManager :: getFileManager();
    File *f = NULL;
    bool readonly = false;
    mstring full(path);
    if (filename) {
        full += filename;
    }

    FRESULT fres = fm->fopen(full.c_str(), FA_READ | FA_WRITE, &f);
    if (fres != FR_OK) {
        fres = fm->fopen(full.c_str(), FA_READ, &f);
        readonly = true;
    }
    if (fres != FR_OK) {
        return fres;
    }
    if (f->get_size() < 512) {
        fm->fclose(f);
        return FR_INVALID_OBJECT;
    }

    unmount();

    xSemaphoreTake(mutex, portMAX_DELAY);
    drive.file = f;
    drive.name = full;
    drive.readonly = readonly;
    const char *ext = strrchr(full.c_str(), '.');
    drive.cf = !(ext && (strcasecmp(ext, ".hdd") == 0));
    drive.detect_geometry();
    present_mask |= 0x01; // interface 0, master
    write_present();
    xSemaphoreGive(mutex);

    printf("IDE64: mounted %s (%u sectors, CHS %u/%u/%u%s)\n", full.c_str(), drive.size,
           drive.cylinders, drive.heads, drive.sectors, readonly ? ", read only" : "");
    return FR_OK;
}

void Ide64 :: set_startup_image(const char *fullpath)
{
    cfg->set_string(CFG_IDE64_IMAGE, fullpath);
    cfg->set_value(CFG_IDE64_MOUNT, 1);
    cfg->write();
}

void Ide64 :: unmount(void)
{
    xSemaphoreTake(mutex, portMAX_DELAY);
    present_mask &= ~0x01;
    write_present();
    if (drive.file) {
        drive.file->sync();
        FileManager :: getFileManager()->fclose(drive.file);
        drive.file = NULL;
        drive.name = "";
        printf("IDE64: unmounted\n");
    }
    xSemaphoreGive(mutex);
}
