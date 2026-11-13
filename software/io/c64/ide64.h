#ifndef IDE64_H
#define IDE64_H

#include <stdint.h>
#include "iomap.h"
#include "config.h"
#include "file.h"
#include "mystring.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

// CPU-side registers of the IDE64 ATA device in the FPGA (ide64_ata.vhd).
// Keep in sync with fpga/cart_slot/vhdl_source/ide64_ata_pkg.vhd.
typedef struct {
    uint8_t features; // 0 R
    uint8_t error;    // 1 R/W
    uint8_t nsect;    // 2 R/W
    uint8_t sector;   // 3 R/W
    uint8_t lcyl;     // 4 R/W
    uint8_t hcyl;     // 5 R/W
    uint8_t select;   // 6 R/W device/head
    uint8_t command;  // 7 R
    uint8_t status;   // 8 R: as the C64 sees it, W: all bits but BSY and DRQ
    uint8_t control;  // 9 W: complete the step (clears BSY), see IDE64_CTRL_*
    uint8_t events;   // A R, W1C
    uint8_t present;  // B R/W: bits 3..0 device present (iface * 2 + dev), bit 7 IRQ enable
    uint8_t devctl;   // C R
    uint8_t iface;    // D R: bit 0 interface, bit 7 IDE64 cart active
    uint8_t pointer;  // E R: word pointer
} ide64_regs_t;

#define IDE64_BUFFER_OFFSET  0x200

#define IDE64_CTRL_ARM       0x01 // start a data phase (pointer = 0, DRQ)
#define IDE64_CTRL_DATA_OUT  0x02 // C64 writes the buffer
#define IDE64_CTRL_MORE      0x04 // data-in: BSY + event when the buffer has been read

#define IDE64_EV_CMD         0x01
#define IDE64_EV_BUF         0x02
#define IDE64_EV_RESET       0x04

#define IDE64_PRESENT_IRQ    0x80

#define ATA_STAT_DRDY        0x40
#define ATA_STAT_ERR         0x01
#define ATA_ERR_UNC          0x40
#define ATA_ERR_WP           0x40
#define ATA_ERR_IDNF         0x10
#define ATA_ERR_ABRT         0x04

class Ide64Drive
{
public:
    File *file;
    mstring name;
    bool readonly;
    bool cf;            // CompactFlash (.cfa) or hard disk (.hdd)
    uint32_t size;      // in sectors
    uint16_t cylinders; // default geometry
    uint8_t  heads;
    uint8_t  sectors;
    uint16_t cur_cylinders; // translation set by INITIALIZE DEVICE PARAMETERS
    uint8_t  cur_heads;
    uint8_t  cur_sectors;

    Ide64Drive() : file(NULL), readonly(false), cf(false), size(0),
                   cylinders(0), heads(0), sectors(0),
                   cur_cylinders(0), cur_heads(0), cur_sectors(0) { }

    void detect_geometry(void);
    void reset_translation(void) {
        cur_cylinders = cylinders;
        cur_heads = heads;
        cur_sectors = sectors;
    }
};

class Ide64 : public ConfigurableObject
{
    volatile ide64_regs_t *regs;
    volatile uint8_t *buffer;
    TaskHandle_t taskHandle;
    SemaphoreHandle_t mutex;
    uint8_t present_mask;
    volatile bool abort_request; // mount/unmount wants the drive
    bool available;
    Ide64Drive drive; // interface 0, master

    uint8_t sector_data[512];

    static void task(void *a);
    static uint8_t irq(void *context);
    void run(void);
    bool wait_buffer(void);
    void lock_drive(void);
    void execute(uint8_t command);
    void complete(uint8_t error);
    bool get_lba(uint32_t &lba);
    void set_command_block(uint32_t lba);
    void cmd_read(void);
    void cmd_write(void);
    void cmd_verify(void);
    void cmd_identify(void);
    void cmd_init_params(void);
    void cmd_set_features(void);
    void write_present(void);
    void copy_to_fpga(const uint8_t *src);
    void copy_from_fpga(uint8_t *dest);
public:
    Ide64();
    void init(void);
    bool is_available(void) { return available; }
    FRESULT mount(const char *path, const char *filename);
    void unmount(void);
    bool is_mounted(void) { return drive.file != NULL; }
    const char *mounted_name(void) { return drive.name.c_str(); }
    void set_startup_image(const char *fullpath);

    static Ide64 *getIde64();
};

#endif
