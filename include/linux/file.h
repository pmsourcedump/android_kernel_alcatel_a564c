/*
 * Wrapper functions for accessing the file_struct fd array.
 */

#ifndef __LINUX_FILE_H
#define __LINUX_FILE_H

#include <linux/compiler.h>
#include <linux/types.h>
#include <linux/posix_types.h>

struct file;

// verify the reserved size for user data partition by zhiling.chen, merged by qian.zhou for PR668979 
#define  LIMIT_USERDATA_SIZE 1

#ifdef LIMIT_USERDATA_SIZE
#define USERDATA_PARTITION_RESERVED_SIZE_TH  10*1024*1024   //50*1024*1024
#endif // LIMIT_USERDATA_SIZE
// verify the reserved size for user data partition by zhiling.chen, merged by qian.zhou for PR668979 
extern void fput(struct file *);

struct file_operations;
struct vfsmount;
struct dentry;
struct path;
extern struct file *alloc_file(struct path *, fmode_t mode,
	const struct file_operations *fop);

static inline void fput_light(struct file *file, int fput_needed)
{
	if (fput_needed)
		fput(file);
}

extern struct file *fget(unsigned int fd);
extern struct file *fget_light(unsigned int fd, int *fput_needed);
extern struct file *fget_raw(unsigned int fd);
extern struct file *fget_raw_light(unsigned int fd, int *fput_needed);
extern void set_close_on_exec(unsigned int fd, int flag);
extern void put_filp(struct file *);
extern int alloc_fd(unsigned start, unsigned flags);
extern int get_unused_fd(void);
#define get_unused_fd_flags(flags) alloc_fd(0, (flags))
extern void put_unused_fd(unsigned int fd);

extern void fd_install(unsigned int fd, struct file *file);

#endif /* __LINUX_FILE_H */
