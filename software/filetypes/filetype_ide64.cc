#include "filetype_ide64.h"
#include "filemanager.h"
#include "userinterface.h"
#include "ide64.h"

// tester instance
FactoryRegistrator<BrowsableDirEntry *, FileType *> tester_ide64(FileType :: getFileTypeFactory(), FileTypeIDE64 :: test_type);

#define IDE64FILE_MOUNT    0x6401
#define IDE64FILE_STARTUP  0x6402
#define IDE64FILE_UNMOUNT  0x6403

FileTypeIDE64 :: FileTypeIDE64(BrowsableDirEntry *node)
{
    this->node = node;
}

FileTypeIDE64 :: ~FileTypeIDE64()
{
}

int FileTypeIDE64 :: fetch_context_items(IndexedList<Action *> &list)
{
    Ide64 *ide = Ide64 :: getIde64();
    if (!ide) {
        return 0;
    }
    list.append(new Action("Mount on IDE64", FileTypeIDE64 :: execute_st, IDE64FILE_MOUNT));
    list.append(new Action("Mount on IDE64 at Startup", FileTypeIDE64 :: execute_st, IDE64FILE_STARTUP));
    int count = 2;
    if (ide->is_mounted()) {
        list.append(new Action("Unmount IDE64 Drive", FileTypeIDE64 :: execute_st, IDE64FILE_UNMOUNT));
        count++;
    }
    return count;
}

FileType *FileTypeIDE64 :: test_type(BrowsableDirEntry *obj)
{
    FileInfo *inf = obj->getInfo();
    if ((strcmp(inf->extension, "HDD") == 0) || (strcmp(inf->extension, "CFA") == 0)) {
        return new FileTypeIDE64(obj);
    }
    return NULL;
}

SubsysResultCode_e FileTypeIDE64 :: execute_st(SubsysCommand *cmd)
{
    Ide64 *ide = Ide64 :: getIde64();
    if (!ide) {
        return SSRET_NOT_IMPLEMENTED;
    }

    if (cmd->functionID == IDE64FILE_UNMOUNT) {
        ide->unmount();
        if (cmd->user_interface) {
            cmd->user_interface->popup("IDE64 drive unmounted", BUTTON_OK);
        }
        return SSRET_OK;
    }

    FRESULT fres = ide->mount(cmd->path.c_str(), cmd->filename.c_str());
    if (fres != FR_OK) {
        if (cmd->user_interface) {
            cmd->user_interface->popup(FileSystem :: get_error_string(fres), BUTTON_OK);
        }
        return SSRET_CANNOT_OPEN_FILE;
    }

    if (cmd->functionID == IDE64FILE_STARTUP) {
        mstring full(cmd->path.c_str());
        full += cmd->filename.c_str();
        ide->set_startup_image(full.c_str());
    }
    return SSRET_OK;
}
