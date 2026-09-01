#ifndef HW_NFS_NFS2_PROTOCOL_H
#define HW_NFS_NFS2_PROTOCOL_H

#define NFS2_NFS_PROGRAM 100003U
#define NFS2_MOUNT_PROGRAM 100005U

#define NFS2_MOUNT_VERSION 1U
#define NFS2_NFS_VERSION 2U

#define NFS2_MAX_DATA (8U * 1024U)
#define NFS2_MAX_PATH 1024U
#define NFS2_MAX_NAME 255U
#define NFS2_FHSIZE 32U
#define NFS2_MAX_HANDLE_RECORDS 65536U
#define NFS2_DUP_CACHE_SIZE 256U
#define NFS2_DUP_CACHE_SECONDS 60U

typedef enum Nfs2MountProcedure {
    NFS2_MOUNT_NULL = 0,
    NFS2_MOUNT_MNT = 1,
    NFS2_MOUNT_DUMP = 2,
    NFS2_MOUNT_UMNT = 3,
    NFS2_MOUNT_UMNTALL = 4,
    NFS2_MOUNT_EXPORT = 5,
} Nfs2MountProcedure;

typedef enum Nfs2Procedure {
    NFS2_NFSPROC_NULL = 0,
    NFS2_NFSPROC_GETATTR = 1,
    NFS2_NFSPROC_SETATTR = 2,
    NFS2_NFSPROC_ROOT = 3,
    NFS2_NFSPROC_LOOKUP = 4,
    NFS2_NFSPROC_READLINK = 5,
    NFS2_NFSPROC_READ = 6,
    NFS2_NFSPROC_WRITECACHE = 7,
    NFS2_NFSPROC_WRITE = 8,
    NFS2_NFSPROC_CREATE = 9,
    NFS2_NFSPROC_REMOVE = 10,
    NFS2_NFSPROC_RENAME = 11,
    NFS2_NFSPROC_LINK = 12,
    NFS2_NFSPROC_SYMLINK = 13,
    NFS2_NFSPROC_MKDIR = 14,
    NFS2_NFSPROC_RMDIR = 15,
    NFS2_NFSPROC_READDIR = 16,
    NFS2_NFSPROC_STATFS = 17,
} Nfs2Procedure;

typedef enum Nfs2Status {
    NFS2_NFS_OK = 0,
    NFS2_NFSERR_PERM = 1,
    NFS2_NFSERR_NOENT = 2,
    NFS2_NFSERR_IO = 5,
    NFS2_NFSERR_NXIO = 6,
    NFS2_NFSERR_ACCES = 13,
    NFS2_NFSERR_EXIST = 17,
    NFS2_NFSERR_NODEV = 19,
    NFS2_NFSERR_NOTDIR = 20,
    NFS2_NFSERR_ISDIR = 21,
    NFS2_NFSERR_FBIG = 27,
    NFS2_NFSERR_NOSPC = 28,
    NFS2_NFSERR_ROFS = 30,
    NFS2_NFSERR_NAMETOOLONG = 63,
    NFS2_NFSERR_NOTEMPTY = 66,
    NFS2_NFSERR_DQUOT = 69,
    NFS2_NFSERR_STALE = 70,
    NFS2_NFSERR_WFLUSH = 99,
} Nfs2Status;

typedef enum Nfs2FileType {
    NFS2_NFNON = 0,
    NFS2_NFREG = 1,
    NFS2_NFDIR = 2,
    NFS2_NFBLK = 3,
    NFS2_NFCHR = 4,
    NFS2_NFLNK = 5,
} Nfs2FileType;

typedef struct Nfs2FileHandle {
    uint8_t bytes[NFS2_FHSIZE];
} Nfs2FileHandle;

typedef struct Nfs2MountMntArgs {
    char path[NFS2_MAX_PATH + 1];
} Nfs2MountMntArgs;

typedef struct Nfs2Diropargs {
    Nfs2FileHandle dir;
    char name[NFS2_MAX_NAME + 1];
} Nfs2Diropargs;

typedef struct Nfs2ReadArgs {
    Nfs2FileHandle file;
    uint32_t offset;
    uint32_t count;
    uint32_t total_count;
} Nfs2ReadArgs;

typedef struct Nfs2WriteArgs {
    Nfs2FileHandle file;
    uint32_t begin_offset;
    uint32_t offset;
    uint32_t total_count;
    const uint8_t *data;
    size_t data_length;
} Nfs2WriteArgs;

#endif
