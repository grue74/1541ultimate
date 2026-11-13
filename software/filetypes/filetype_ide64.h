#ifndef FILETYPE_IDE64_H
#define FILETYPE_IDE64_H

#include "filetypes.h"
#include "browsable_root.h"

// IDE64 disk images (.hdd, .cfa), as used by VICE
class FileTypeIDE64 : public FileType
{
    BrowsableDirEntry *node;
public:
    FileTypeIDE64(BrowsableDirEntry *par);
    ~FileTypeIDE64();

    int fetch_context_items(IndexedList<Action *> &list);
    static FileType *test_type(BrowsableDirEntry *obj);
    static SubsysResultCode_e execute_st(SubsysCommand *cmd);
};

#endif
