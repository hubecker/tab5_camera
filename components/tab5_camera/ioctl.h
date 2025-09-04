/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: ESPRESSIF MIT
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* --------------------------------------------------------------------
 *  Définitions de base – protégées contre les doubles définitions.
 * -------------------------------------------------------------------- */
#ifndef _IOC_NRBITS
#define _IOC_NRBITS   8
#endif

#ifndef _IOC_TYPEBITS
#define _IOC_TYPEBITS 8
#endif

#ifndef _IOC_SIZEBITS
#define _IOC_SIZEBITS 14
#endif

#ifndef _IOC_DIRBITS
#define _IOC_DIRBITS  2
#endif

/* --------------------------------------------------------------------
 *  Valeurs de direction (NONE, WRITE, READ) – mêmes protections.
 * -------------------------------------------------------------------- */
#ifndef _IOC_NONE
#define _IOC_NONE 0U
#endif

#ifndef _IOC_WRITE
#define _IOC_WRITE 1U
#endif

#ifndef _IOC_READ
#define _IOC_READ 2U
#endif

/* --------------------------------------------------------------------
 *  Masques calculés à partir des tailles de champ.
 * -------------------------------------------------------------------- */
#ifndef _IOC_NRMASK
#define _IOC_NRMASK   ((1U << _IOC_NRBITS) - 1U)
#endif

#ifndef _IOC_TYPEMASK
#define _IOC_TYPEMASK ((1U << _IOC_TYPEBITS) - 1U)
#endif

#ifndef _IOC_SIZEMASK
#define _IOC_SIZEMASK ((1U << _IOC_SIZEBITS) - 1U)
#endif

#ifndef _IOC_DIRMASK
#define _IOC_DIRMASK  ((1U << _IOC_DIRBITS) - 1U)
#endif

/* --------------------------------------------------------------------
 *  Décalages des champs dans le mot de commande.
 * -------------------------------------------------------------------- */
#ifndef _IOC_NRSHIFT
#define _IOC_NRSHIFT   0U
#endif

#ifndef _IOC_TYPESHIFT
#define _IOC_TYPESHIFT (_IOC_NRSHIFT + _IOC_NRBITS)
#endif

#ifndef _IOC_SIZESHIFT
#define _IOC_SIZESHIFT (_IOC_TYPESHIFT + _IOC_TYPEBITS)
#endif

#ifndef _IOC_DIRSHIFT
#define _IOC_DIRSHIFT  (_IOC_SIZESHIFT + _IOC_SIZEBITS)
#endif

/* --------------------------------------------------------------------
 *  Macro principale qui construit le code ioctl.
 * -------------------------------------------------------------------- */
#ifndef _IOC
#define _IOC(dir, type, nr, size)                                    \
  (((dir)  << _IOC_DIRSHIFT)  |                                      \
   ((type) << _IOC_TYPESHIFT) |                                      \
   ((nr)   << _IOC_NRSHIFT)   |                                      \
   ((size) << _IOC_SIZESHIFT))
#endif

/* --------------------------------------------------------------------
 *  Helper pour obtenir la taille d’un type (comme dans le kernel Linux).
 * -------------------------------------------------------------------- */
#ifndef _IOC_TYPECHECK
#define _IOC_TYPECHECK(t) (sizeof(t))
#endif

/* --------------------------------------------------------------------
 *  Macros « convenance » utilisées par le driver.
 * -------------------------------------------------------------------- */
#ifndef _IO
#define _IO(type, nr)          _IOC(_IOC_NONE,  (type), (nr), 0)
#endif

#ifndef _IOR
#define _IOR(type, nr, size)   _IOC(_IOC_READ,  (type), (nr), _IOC_TYPECHECK(size))
#endif

#ifndef _IOW
#define _IOW(type, nr, size)   _IOC(_IOC_WRITE, (type), (nr), _IOC_TYPECHECK(size))
#endif

#ifndef _IOWR
#define _IOWR(type, nr, size)  _IOC(_IOC_READ | _IOC_WRITE, (type), (nr), _IOC_TYPECHECK(size))
#endif

#ifdef __cplusplus
}
#endif
