/**************************************************************************
 *
 * Copyright (c) 2009, 2010 Marvell International Ltd.
 *
 * This file is part of GNU program.
 *
 * GNU program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 2 of the License, or (at your
 * option) any later version.
 *
 * GNU program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.
 *
 * If not, see http://www.gnu.org/licenses/old-licenses/gpl-2.0.html
 *
 *************************************************************************/

#ifndef __LINUX_USB_GADGET_PXA_COMMON_H
#define __LINUX_USB_GADGET_PXA_COMMON_H

enum ep0_state {
	EP0_IDLE,
	EP0_IN_DATA_PHASE,
	EP0_OUT_DATA_PHASE,
	EP0_STALL,
	EP0_IN_FAKE,
	EP0_NO_ACTION
};

#define DMSG(stuff...)  pr_debug("udc: " stuff)

#define VBUS_LOW	(0)
#define VBUS_HIGH	(1<<0)
#define VBUS_CHARGE	(1<<1)
#define VBUS_SRP	(1<<2)

#endif /* __LINUX_USB_GADGET_PXA_COMMON_H */
