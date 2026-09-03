/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Clean-room description of the historical NetInfo RPC wire contract.
 *
 * The constants and XDR declarations are transcribed from the pinned,
 * preserved NeXT libc-34.1 protocol specifications (not generated C output):
 *
 *   https://github.com/johnsonjh/NeXTSrc/blob/ff846608a76ab2fbbb86e8a14c52ac85332f9786/libc-34.1/usr.include/netinfo/ni_prot.x
 *   https://github.com/johnsonjh/NeXTSrc/blob/ff846608a76ab2fbbb86e8a14c52ac85332f9786/libc-34.1/usr.include/netinfo/nibind_prot.x
 *
 * The corresponding source-file SHA256 values are
 * 8dc940ec6da326fb5a949c53c8d7957be9dee3bc0ae5be9b13c22175532c77ce
 * (ni_prot.x) and
 * 1b489f05f9842cc390e28321f93f4eb75c1d6d99a8e7cf98ab5b5882da583c1a
 * (nibind_prot.x).  They are cross-checked against Apple's published
 * Libinfo-221 specifications (not generated C output):
 *
 *   https://github.com/apple-oss-distributions/Libinfo/blob/Libinfo-221/netinfo.subproj/nibind_prot.x
 *   https://github.com/apple-oss-distributions/Libinfo/blob/Libinfo-221/netinfo.subproj/ni_prot.x
 *
 * XDR's four-byte units, counted strings/arrays, optional data, and ONC RPC
 * message framing follow RFC 1832 and RFC 1831:
 *
 *   https://www.rfc-editor.org/rfc/rfc1832.html
 *   https://www.rfc-editor.org/rfc/rfc1831.html
 *
 * The fixed status contract follows the preserved NeXT source; the later
 * Libinfo-221 ni_prot.x revision appends NI_MASTERBUSY, NI_INVALIDDOMAIN, and
 * NI_BADOP.  This header deliberately does not include generated code or
 * Apple's implementation.  The C structures below are ownership-safe
 * QEMU-side representations of the published types.
 */
#ifndef HW_NETINFO_NETINFO_PROTOCOL_H
#define HW_NETINFO_NETINFO_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Published ONC RPC program/version numbers. */
#define NIBIND_PROG 200100001U
#define NIBIND_VERS 1U
#define NI_PROG 200100000U
#define NI_VERS 2U

/* Fixed guest-visible service ports used by the read-only server. */
#define NIBIND_UDP_PORT 659U
#define NIBIND_TCP_PORT 661U
#define NI_UDP_PORT 660U
#define NI_TCP_PORT 662U

/* Historical maxima from ni_prot.x and nibind_prot.x. */
#define NIBIND_MAXREGS 32U
#define NI_NAME_MAXLEN 65535U
#define NI_NAMELIST_MAXLEN 65535U
#define NI_PROPLIST_MAXLEN 65535U
#define NI_IDLIST_MAXLEN 1048576U

/* Lower service limits.  These are policy limits, not wire maxima. */
#define NI_SERVICE_MAX_NODES 4096U
#define NI_SERVICE_MAX_PROPERTIES 256U
#define NI_SERVICE_MAX_VALUES 1024U
#define NI_SERVICE_MAX_NAME 4096U
#define NI_SERVICE_MAX_TCP_RECORD (1U * 1024U * 1024U)
#define NI_SERVICE_MAX_DEPTH 128U

/* ni_object_list has no historical count bound; cap this linked wire list. */
#define NI_OBJECT_LIST_MAXLEN 4096U

/* Binder procedure numbers from nibind_prot.x. */
typedef enum NiBindProcedure {
    NIBIND_PROC_PING = 0,
    NIBIND_PROC_REGISTER = 1,
    NIBIND_PROC_UNREGISTER = 2,
    NIBIND_PROC_GETREGISTER = 3,
    NIBIND_PROC_LISTREG = 4,
    NIBIND_PROC_CREATEMASTER = 5,
    NIBIND_PROC_CREATECLONE = 6,
    NIBIND_PROC_DESTROYDOMAIN = 7,
    NIBIND_PROC_BIND = 8,
} NiBindProcedure;

#define NIBIND_PING NIBIND_PROC_PING
#define NIBIND_REGISTER NIBIND_PROC_REGISTER
#define NIBIND_UNREGISTER NIBIND_PROC_UNREGISTER
#define NIBIND_GETREGISTER NIBIND_PROC_GETREGISTER
#define NIBIND_LISTREG NIBIND_PROC_LISTREG
#define NIBIND_CREATEMASTER NIBIND_PROC_CREATEMASTER
#define NIBIND_CREATECLONE NIBIND_PROC_CREATECLONE
#define NIBIND_DESTROYDOMAIN NIBIND_PROC_DESTROYDOMAIN
#define NIBIND_BIND NIBIND_PROC_BIND

/* Database procedure numbers from ni_prot.x. */
typedef enum NiProcedure {
    NI_PROC_PING = 0,
    NI_PROC_STATISTICS = 1,
    NI_PROC_ROOT = 2,
    NI_PROC_SELF = 3,
    NI_PROC_PARENT = 4,
    NI_PROC_CREATE = 5,
    NI_PROC_DESTROY = 6,
    NI_PROC_READ = 7,
    NI_PROC_WRITE = 8,
    NI_PROC_CHILDREN = 9,
    NI_PROC_LOOKUP = 10,
    NI_PROC_LIST = 11,
    NI_PROC_CREATEPROP = 12,
    NI_PROC_DESTROYPROP = 13,
    NI_PROC_READPROP = 14,
    NI_PROC_WRITEPROP = 15,
    NI_PROC_RENAMEPROP = 16,
    NI_PROC_LISTPROPS = 17,
    NI_PROC_CREATENAME = 18,
    NI_PROC_DESTROYNAME = 19,
    NI_PROC_READNAME = 20,
    NI_PROC_WRITENAME = 21,
    NI_PROC_RPARENT = 22,
    NI_PROC_LISTALL = 23,
    NI_PROC_BIND = 24,
    NI_PROC_READALL = 25,
    NI_PROC_CRASHED = 26,
    NI_PROC_RESYNC = 27,
    NI_PROC_LOOKUPREAD = 28,
} NiProcedure;

#define NI_PING NI_PROC_PING
#define NI_STATISTICS NI_PROC_STATISTICS
#define NI_ROOT NI_PROC_ROOT
#define NI_SELF NI_PROC_SELF
#define NI_PARENT NI_PROC_PARENT
#define NI_CREATE NI_PROC_CREATE
#define NI_DESTROY NI_PROC_DESTROY
#define NI_READ NI_PROC_READ
#define NI_WRITE NI_PROC_WRITE
#define NI_CHILDREN NI_PROC_CHILDREN
#define NI_LOOKUP NI_PROC_LOOKUP
#define NI_LIST NI_PROC_LIST
#define NI_CREATEPROP NI_PROC_CREATEPROP
#define NI_DESTROYPROP NI_PROC_DESTROYPROP
#define NI_READPROP NI_PROC_READPROP
#define NI_WRITEPROP NI_PROC_WRITEPROP
#define NI_RENAMEPROP NI_PROC_RENAMEPROP
#define NI_LISTPROPS NI_PROC_LISTPROPS
#define NI_CREATENAME NI_PROC_CREATENAME
#define NI_DESTROYNAME NI_PROC_DESTROYNAME
#define NI_READNAME NI_PROC_READNAME
#define NI_WRITENAME NI_PROC_WRITENAME
#define NI_RPARENT NI_PROC_RPARENT
#define NI_LISTALL NI_PROC_LISTALL
#define NI_BIND NI_PROC_BIND
#define NI_READALL NI_PROC_READALL
#define NI_CRASHED NI_PROC_CRASHED
#define NI_RESYNC NI_PROC_RESYNC
#define NI_LOOKUPREAD NI_PROC_LOOKUPREAD

/*
 * The published database declarations spell these procedure names with a
 * leading underscore; retain both spellings for callers.
 */
#define _NI_PING NI_PROC_PING
#define _NI_STATISTICS NI_PROC_STATISTICS
#define _NI_ROOT NI_PROC_ROOT
#define _NI_SELF NI_PROC_SELF
#define _NI_PARENT NI_PROC_PARENT
#define _NI_CREATE NI_PROC_CREATE
#define _NI_DESTROY NI_PROC_DESTROY
#define _NI_READ NI_PROC_READ
#define _NI_WRITE NI_PROC_WRITE
#define _NI_CHILDREN NI_PROC_CHILDREN
#define _NI_LOOKUP NI_PROC_LOOKUP
#define _NI_LIST NI_PROC_LIST
#define _NI_CREATEPROP NI_PROC_CREATEPROP
#define _NI_DESTROYPROP NI_PROC_DESTROYPROP
#define _NI_READPROP NI_PROC_READPROP
#define _NI_WRITEPROP NI_PROC_WRITEPROP
#define _NI_RENAMEPROP NI_PROC_RENAMEPROP
#define _NI_LISTPROPS NI_PROC_LISTPROPS
#define _NI_CREATENAME NI_PROC_CREATENAME
#define _NI_DESTROYNAME NI_PROC_DESTROYNAME
#define _NI_READNAME NI_PROC_READNAME
#define _NI_WRITENAME NI_PROC_WRITENAME
#define _NI_RPARENT NI_PROC_RPARENT
#define _NI_LISTALL NI_PROC_LISTALL
#define _NI_BIND NI_PROC_BIND
#define _NI_READALL NI_PROC_READALL
#define _NI_CRASHED NI_PROC_CRASHED
#define _NI_RESYNC NI_PROC_RESYNC
#define _NI_LOOKUPREAD NI_PROC_LOOKUPREAD

/*
 * The fixed NeXTSTEP wire contract uses values 0 through 21 plus the explicit
 * local-error value 9999, as in the preserved libc-34.1 NeXT distribution's
 * netinfo/ni_prot.x.  Later Apple Libinfo revisions append values 22 through
 * 24; those are intentionally outside this compatibility contract.
 */
typedef enum NiStatus {
    NI_OK = 0,
    NI_BADID = 1,
    NI_STALE = 2,
    NI_NOSPACE = 3,
    NI_PERM = 4,
    NI_NODIR = 5,
    NI_NOPROP = 6,
    NI_NONAME = 7,
    NI_NOTEMPTY = 8,
    NI_UNRELATED = 9,
    NI_SERIAL = 10,
    NI_NETROOT = 11,
    NI_NORESPONSE = 12,
    NI_RDONLY = 13,
    NI_SYSTEMERR = 14,
    NI_ALIVE = 15,
    NI_NOTMASTER = 16,
    NI_CANTFINDADDRESS = 17,
    NI_DUPTAG = 18,
    NI_NOTAG = 19,
    NI_AUTHERROR = 20,
    NI_NOUSER = 21,
    NI_FAILED = 9999,
} NiStatus;

typedef uint32_t NiIndex;
typedef char *NiName;

typedef struct NiId {
    NiIndex nii_object;
    NiIndex nii_instance;
} NiId;

typedef struct NiNameList {
    size_t count;
    NiName *values;
} NiNameList;

typedef struct NiProperty {
    NiName name;
    NiNameList values;
} NiProperty;

typedef struct NiPropertyList {
    size_t count;
    NiProperty *properties;
} NiPropertyList;

typedef struct NiIdList {
    size_t count;
    NiIndex *values;
} NiIdList;

typedef struct NiObject {
    NiId id;
    NiPropertyList properties;
    NiIndex parent;
    NiIdList children;
} NiObject;

typedef struct NiEntry {
    NiIndex id;
    bool has_names;
    NiNameList names;
} NiEntry;

typedef struct NiEntryList {
    size_t count;
    NiEntry *entries;
} NiEntryList;

typedef struct NiBindAddrInfo {
    uint32_t udp_port;
    uint32_t tcp_port;
} NiBindAddrInfo;

typedef struct NiBindRegistration {
    NiName tag;
    NiBindAddrInfo addrs;
} NiBindRegistration;

typedef struct NiBindCloneArgs {
    NiName tag;
    NiName master_name;
    uint32_t master_addr;
    NiName master_tag;
} NiBindCloneArgs;

typedef struct NiBindArgs {
    uint32_t client_addr;
    NiName client_tag;
    NiName server_tag;
} NiBindArgs;

typedef struct NiBindGetRegisterResult {
    NiStatus status;
    NiBindAddrInfo addrs;
} NiBindGetRegisterResult;

typedef struct NiBindListRegResult {
    NiStatus status;
    size_t count;
    NiBindRegistration *registrations;
} NiBindListRegResult;

typedef struct NiIdResult {
    NiStatus status;
    bool has_id;
    NiId id;
} NiIdResult;
typedef NiIdResult NiIdRes;

typedef struct NiParentStuff {
    NiIndex object_id;
    NiId self_id;
} NiParentStuff;

typedef struct NiParentResult {
    NiStatus status;
    NiParentStuff stuff;
} NiParentResult;

typedef struct NiChildrenStuff {
    NiIdList children;
    NiId self_id;
} NiChildrenStuff;

typedef struct NiChildrenResult {
    NiStatus status;
    NiChildrenStuff stuff;
} NiChildrenResult;

typedef struct NiEntryStuff {
    NiEntryList entries;
    NiId self_id;
} NiEntryStuff;

typedef struct NiListResult {
    NiStatus status;
    NiEntryStuff stuff;
} NiListResult;

typedef struct NiPropertyListStuff {
    NiId id;
    NiPropertyList props;
} NiPropertyListStuff;

typedef struct NiCreateArgs {
    NiId id;
    NiPropertyList props;
    NiIndex where;
    bool has_target_id;
    NiId target_id;
} NiCreateArgs;

typedef struct NiPropertyListResult {
    NiStatus status;
    NiPropertyListStuff stuff;
} NiPropertyListResult;

typedef struct NiCreateStuff {
    NiId id;
    NiId self_id;
} NiCreateStuff;

typedef struct NiCreateResult {
    NiStatus status;
    NiCreateStuff stuff;
} NiCreateResult;

typedef struct NiDestroyArgs {
    NiId parent_id;
    NiId self_id;
} NiDestroyArgs;

typedef struct NiLookupArgs {
    NiId id;
    NiName key;
    NiName value;
} NiLookupArgs;

typedef struct NiLookupStuff {
    NiIdList idlist;
    NiId self_id;
} NiLookupStuff;

typedef struct NiLookupResult {
    NiStatus status;
    NiLookupStuff stuff;
} NiLookupResult;

typedef struct NiNameArgs {
    NiId id;
    NiName name;
} NiNameArgs;

typedef struct NiCreatePropArgs {
    NiId id;
    NiProperty prop;
    NiIndex where;
} NiCreatePropArgs;

typedef struct NiWritePropArgs {
    NiId id;
    NiIndex prop_index;
    NiNameList values;
} NiWritePropArgs;

typedef struct NiPropArgs {
    NiId id;
    NiIndex prop_index;
} NiPropArgs;

typedef struct NiNameListStuff {
    NiNameList values;
    NiId self_id;
} NiNameListStuff;

typedef struct NiNameListResult {
    NiStatus status;
    NiNameListStuff stuff;
} NiNameListResult;

typedef struct NiPropNameArgs {
    NiId id;
    NiIndex prop_index;
    NiName name;
} NiPropNameArgs;

typedef struct NiCreateNameArgs {
    NiId id;
    NiIndex prop_index;
    NiName name;
    NiIndex where;
} NiCreateNameArgs;

typedef struct NiNameIndexArgs {
    NiId id;
    NiIndex prop_index;
    NiIndex name_index;
} NiNameIndexArgs;

typedef struct NiWriteNameArgs {
    NiId id;
    NiIndex prop_index;
    NiIndex name_index;
    NiName name;
} NiWriteNameArgs;

typedef struct NiReadNameStuff {
    NiId id;
    NiName name;
} NiReadNameStuff;

typedef struct NiReadNameResult {
    NiStatus status;
    NiReadNameStuff stuff;
} NiReadNameResult;

typedef struct NiBinding {
    NiName tag;
    uint32_t addr;
} NiBinding;

typedef struct NiRParentResult {
    NiStatus status;
    NiBinding binding;
} NiRParentResult;

typedef struct NiObjectNode NiObjectNode;
struct NiObjectNode {
    NiObject object;
    NiObjectNode *next;
};

typedef struct NiObjectList {
    NiObjectNode *head;
    size_t count;
} NiObjectList;

typedef struct NiReadAllStuff {
    uint32_t checksum;
    NiIndex highestid;
    NiObjectList list;
} NiReadAllStuff;

typedef struct NiReadAllResult {
    NiStatus status;
    NiReadAllStuff stuff;
} NiReadAllResult;

typedef struct NiPropertyListArray {
    size_t count;
    NiPropertyList *entries;
} NiPropertyListArray;

typedef struct NiListAllStuff {
    NiId self_id;
    NiPropertyListArray entries;
} NiListAllStuff;

typedef struct NiListAllResult {
    NiStatus status;
    NiListAllStuff stuff;
} NiListAllResult;

#endif /* HW_NETINFO_NETINFO_PROTOCOL_H */
