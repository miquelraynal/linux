/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Silvaco Autonomous Slave driver
 *
 * Copyright 2019 Silvaco Inc.
 *
 * Conor Culhane <conor.culhane@silvaco.com>
 * Ron Werner <ron.werner@silvaco.com>
 */

#ifndef SVC_AUTS_H
#define SVC_AUTS_H


#include <linux/device.h>
#include <linux/regmap.h>
#include <linux/types.h>
#include <linux/compiler.h>


#define SVC_I3C_AUTS_IOCTL_MAGIC 'S' // (S)ilvaco

// Configure I3C Target Register
#define SVC_I3C_IOCTL_TARGET_REG _IOW(SVC_I3C_AUTS_IOCTL_MAGIC, 'a', __u8)

// Poll I3C Slave for IBI status (and read the IBI data if available)
#define SVC_I3c_IOCTL_POLL_IBI _IOWR(SVC_I3C_AUTS_IOCTL_MAGIC, 'b', __u8)



#endif // SVC_AUTS_H
