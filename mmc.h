
/**
 * @file    mmc.h
 * @brief
 *
 * @addtogroup MMC
 * @{
 */

#ifndef _MMC_H_
#define _MMC_H_

#include "ff.h"


/*===========================================================================*/
/* FatFs related.                                                            */
/*===========================================================================*/

/**
 * @brief FS object.
 */
static FATFS MMC_FS;

FRESULT scan_files(BaseSequentialStream *chp, char *path);
void cmd_mount(BaseSequentialStream *chp, int argc, char *argv[]);
void cmd_unmount(BaseSequentialStream *chp, int argc, char *argv[]);
void cmd_free(BaseSequentialStream *chp, int argc, char *argv[]);
void cmd_tree(BaseSequentialStream *chp, int argc, char *argv[]);
void cmd_mkfs(BaseSequentialStream *chp, int argc, char *argv[]);
void cmd_setlabel(BaseSequentialStream *chp, int argc, char *argv[]);
void cmd_getlabel(BaseSequentialStream *chp, int argc, char *argv[]);
void cmd_hello(BaseSequentialStream *chp, int argc, char *argv[]);
void cmd_mkdir(BaseSequentialStream *chp, int argc, char *argv[]);
void cmd_cat(BaseSequentialStream *chp, int argc, char *argv[]);
void verbose_error(BaseSequentialStream *chp, FRESULT err);
char* fresult_str(FRESULT stat);


#endif /* _MMC_H_ */

/** @} */
