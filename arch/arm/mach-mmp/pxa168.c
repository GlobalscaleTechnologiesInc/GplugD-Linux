/*
 *  linux/arch/arm/mach-mmp/pxa168.c
 *
 *  Code specific to PXA168
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/platform_device.h>

#include <asm/mach/time.h>
#include <asm/system_misc.h>
#include <mach/addr-map.h>
#include <mach/cputype.h>
#include <mach/regs-apbc.h>
#include <mach/regs-apmu.h>
#include <mach/irqs.h>
#include <mach/dma.h>
#include <mach/devices.h>
#include <mach/mfp.h>
#include <linux/dma-mapping.h>
#include <mach/pxa168.h>

#include <plat/pxa_u2o.h>
#include <linux/delay.h>

#include "common.h"
#include "clock.h"

#define MFPR_VIRT_BASE	(APB_VIRT_BASE + 0x1e000)

static struct mfp_addr_map pxa168_mfp_addr_map[] __initdata =
{
	MFP_ADDR_X(GPIO0,   GPIO36,  0x04c),
	MFP_ADDR_X(GPIO37,  GPIO55,  0x000),
	MFP_ADDR_X(GPIO56,  GPIO123, 0x0e0),
	MFP_ADDR_X(GPIO124, GPIO127, 0x0f4),

	MFP_ADDR_END,
};

void __init pxa168_init_irq(void)
{
	icu_init_irq();
}
static void pseudo_clk_enable(struct clk *clk)
{
}

static void pseudo_clk_disable(struct clk *clk)
{
}

struct clkops pseudo_clk_ops = {
        .enable         = pseudo_clk_enable,
        .disable        = pseudo_clk_disable,
};

static void u2h_clk_enable(struct clk *clk)
{
	u32 tmp = __raw_readl(clk->clk_rst);

	tmp |= 0x12;
	__raw_writel(tmp, clk->clk_rst);
}

static void u2h_clk_disable(struct clk *clk)
{
	u32 tmp = __raw_readl(clk->clk_rst);

	tmp &= ~0x12;
	__raw_writel(tmp, clk->clk_rst);
}

struct clkops u2h_clk_ops = {
	.enable		= u2h_clk_enable,
	.disable	= u2h_clk_disable,
};

static void u2o_clk_enable(struct clk *clk)
{
	u32 tmp = __raw_readl(clk->clk_rst);
	tmp |= 0x9;
	__raw_writel(tmp, clk->clk_rst);
}

static void u2o_clk_disable(struct clk *clk)
{
	u32 tmp = __raw_readl(clk->clk_rst);
	tmp &= ~0x9;
	__raw_writel(tmp, clk->clk_rst);
}

struct clkops u2o_clk_ops = {
	.enable		= u2o_clk_enable,
	.disable	= u2o_clk_disable,
};

static void sdh1_clk_enable(struct clk *clk)
{
	/* Bits 3 & 0 in registers for host 0 should be set for host 1 also */
	__raw_writel(__raw_readl(APMU_SDH0) | 0x9, APMU_SDH0);

	__raw_writel(__raw_readl(clk->clk_rst) | clk->enable_val, clk->clk_rst);
}

static void sdh2_clk_enable(struct clk *clk)
{
	/* Bits 3 & 0 in registers for host 2 should be set for host 3 also */
	__raw_writel(__raw_readl(APMU_SDH2) | 0x9, APMU_SDH2);

	__raw_writel(__raw_readl(clk->clk_rst) | clk->enable_val, clk->clk_rst);
}

static void sdh_clk_disable(struct clk *clk)
{
	__raw_writel(__raw_readl(clk->clk_rst) & ~clk->enable_val,
			clk->clk_rst);
}

/* Block 1 for controller 0 & 1 */
struct clkops sdh1_clk_ops = {
	.enable		= sdh1_clk_enable,
	.disable	= sdh_clk_disable,
};

/* Block 2 for controller 2 & 3 */
struct clkops sdh2_clk_ops = {
	.enable		= sdh2_clk_enable,
	.disable	= sdh_clk_disable,
};

/* APB peripheral clocks */
static APBC_CLK(uart1, PXA168_UART1, 1, 14745600);
static APBC_CLK(uart2, PXA168_UART2, 1, 14745600);
static APBC_CLK(uart3, PXA168_UART3, 1, 14745600);
static APBC_CLK(twsi0, PXA168_TWSI0, 1, 33000000);
static APBC_CLK(twsi1, PXA168_TWSI1, 1, 33000000);
static APBC_CLK(pwm1, PXA168_PWM1, 1, 13000000);
static APBC_CLK(pwm2, PXA168_PWM2, 1, 13000000);
static APBC_CLK(pwm3, PXA168_PWM3, 1, 13000000);
static APBC_CLK(pwm4, PXA168_PWM4, 1, 13000000);
static APBC_CLK(ssp1, PXA168_SSP1, 4, 0);
static APBC_CLK(ssp2, PXA168_SSP2,  0, 6500000);  //bytim-tian
static APBC_CLK(ssp3, PXA168_SSP3, 4, 0);
static APBC_CLK(ssp4, PXA168_SSP4, 4, 0);
static APBC_CLK(ssp5, PXA168_SSP5, 4, 0);
static APBC_CLK(gpio, PXA168_GPIO, 0, 13000000);
static APBC_CLK(keypad, PXA168_KPC, 0, 32000);
static APBC_CLK(rtc, PXA168_RTC, 8, 32768);

static APMU_CLK(nand, NAND, 0x19b, 156000000);
static APMU_CLK(lcd, LCD, 0x7f, 312000000);
static APMU_CLK(eth, ETH, 0x09, 0);

static APMU_CLK_OPS(sdh1, SDH0, 0x12, 48000000, &sdh1_clk_ops); //bytim-tian
static APMU_CLK_OPS(sdh2, SDH1, 0x12, 48000000, &sdh1_clk_ops);
static APMU_CLK_OPS(sdh3, SDH2, 0x12, 48000000, &sdh2_clk_ops);
static APMU_CLK_OPS(sdh4, SDH3, 0x12, 48000000, &sdh2_clk_ops);
static APMU_CLK_OPS(u2h, USB, 0x0, 480000000, &u2h_clk_ops);	/* 480MHz, AXICLK */ //bytim-tian
static APMU_CLK_OPS(u2o, USB, 0x0, 480000000, &u2o_clk_ops);	/* 480MHz, AXICLK */ //bytim-tian
static APMU_CLK_OPS(u2oehci, USB, 0x0, 480000000, &pseudo_clk_ops);	/* 480MHz, AXICLK */ //bytim-tian
static APMU_CLK_OPS(u2ogadget, USB, 0x0, 480000000, &pseudo_clk_ops);	/* 480MHz, AXICLK */ //bytim-tian


/* device and clock bindings */
static struct clk_lookup pxa168_clkregs[] = {
	INIT_CLKREG(&clk_uart1, "pxa2xx-uart.0", NULL),
	INIT_CLKREG(&clk_uart2, "pxa2xx-uart.1", NULL),
	INIT_CLKREG(&clk_uart3, "pxa2xx-uart.2", NULL),
	INIT_CLKREG(&clk_twsi0, "pxa2xx-i2c.0", NULL),
	INIT_CLKREG(&clk_twsi1, "pxa2xx-i2c.1", NULL),
	INIT_CLKREG(&clk_pwm1, "pxa168-pwm.0", NULL),
	INIT_CLKREG(&clk_pwm2, "pxa168-pwm.1", NULL),
	INIT_CLKREG(&clk_pwm3, "pxa168-pwm.2", NULL),
	INIT_CLKREG(&clk_pwm4, "pxa168-pwm.3", NULL),
	INIT_CLKREG(&clk_ssp1, "pxa168-ssp.0", NULL),
	INIT_CLKREG(&clk_ssp2, "pxa168-ssp.1", NULL),
	INIT_CLKREG(&clk_ssp3, "pxa168-ssp.2", NULL),
	INIT_CLKREG(&clk_ssp4, "pxa168-ssp.3", NULL),
	INIT_CLKREG(&clk_ssp5, "pxa168-ssp.4", NULL),
	INIT_CLKREG(&clk_nand, "pxa3xx-nand", NULL),
	INIT_CLKREG(&clk_lcd, "pxa168-fb", NULL),
	INIT_CLKREG(&clk_gpio, "pxa-gpio", NULL),
	INIT_CLKREG(&clk_keypad, "pxa27x-keypad", NULL),
	INIT_CLKREG(&clk_eth, "pxa168-eth", "MFUCLK"),
	//INIT_CLKREG(&clk_usb, "pxa168-ehci", "PXA168-USBCLK"),  
	INIT_CLKREG(&clk_u2h, NULL, "PXA168-U2HCLK"),  //bytim-tian
	INIT_CLKREG(&clk_u2o, NULL, "PXA168-U2OCLK"),  //bytim-tian
	INIT_CLKREG(&clk_u2oehci, NULL, "PXA168-U2OEHCICLK"),  //bytim-tian
	INIT_CLKREG(&clk_u2ogadget, NULL, "PXA168-U2OGADGETCLK"),  //bytim-tian
	INIT_CLKREG(&clk_rtc, "sa1100-rtc", NULL),
	INIT_CLKREG(&clk_sdh1, "sdhci-pxav2.0", "PXA-SDHCLK"),
	INIT_CLKREG(&clk_sdh2, "sdhci-pxav2.1", "PXA-SDHCLK"),
	INIT_CLKREG(&clk_sdh3, "sdhci-pxav2.2", "PXA-SDHCLK"),
	INIT_CLKREG(&clk_sdh4, "sdhci-pxav2.3", "PXA-SDHCLK"),
};

static int __init pxa168_init(void)
{
	if (cpu_is_pxa168()) {
		mfp_init_base(MFPR_VIRT_BASE);
		mfp_init_addr(pxa168_mfp_addr_map);
		pxa_init_dma(IRQ_PXA168_DMA_INT0, 32);
		clkdev_add_table(ARRAY_AND_SIZE(pxa168_clkregs));
	}

	return 0;
}
postcore_initcall(pxa168_init);

/* system timer - clock enabled, 3.25MHz */
#define TIMER_CLK_RST	(APBC_APBCLK | APBC_FNCLK | APBC_FNCLKSEL(3))

static void __init pxa168_timer_init(void)
{
	/* this is early, we have to initialize the CCU registers by
	 * ourselves instead of using clk_* API. Clock rate is defined
	 * by APBC_TIMERS_CLK_RST (3.25MHz) and enabled free-running
	 */
	__raw_writel(APBC_APBCLK | APBC_RST, APBC_PXA168_TIMERS);

	/* 3.25MHz, bus/functional clock enabled, release reset */
	__raw_writel(TIMER_CLK_RST, APBC_PXA168_TIMERS);

	timer_init(IRQ_PXA168_TIMER1);
}

struct sys_timer pxa168_timer = {
	.init	= pxa168_timer_init,
};

void pxa168_clear_keypad_wakeup(void)
{
	uint32_t val;
	uint32_t mask = APMU_PXA168_KP_WAKE_CLR;

	/* wake event clear is needed in order to clear keypad interrupt */
	val = __raw_readl(APMU_WAKE_CLR);
	__raw_writel(val |  mask, APMU_WAKE_CLR);
}

/* on-chip devices */
PXA168_DEVICE(uart1, "pxa2xx-uart", 0, UART1, 0xd4017000, 0x30, 21, 22);
PXA168_DEVICE(uart2, "pxa2xx-uart", 1, UART2, 0xd4018000, 0x30, 23, 24);
PXA168_DEVICE(uart3, "pxa2xx-uart", 2, UART3, 0xd4026000, 0x30, 23, 24);
PXA168_DEVICE(twsi0, "pxa2xx-i2c", 0, TWSI0, 0xd4011000, 0x28);
PXA168_DEVICE(twsi1, "pxa2xx-i2c", 1, TWSI1, 0xd4025000, 0x28);
PXA168_DEVICE(pwm1, "pxa168-pwm", 0, NONE, 0xd401a000, 0x10);
PXA168_DEVICE(pwm2, "pxa168-pwm", 1, NONE, 0xd401a400, 0x10);
PXA168_DEVICE(pwm3, "pxa168-pwm", 2, NONE, 0xd401a800, 0x10);
PXA168_DEVICE(pwm4, "pxa168-pwm", 3, NONE, 0xd401ac00, 0x10);
PXA168_DEVICE(nand, "pxa3xx-nand", -1, NAND, 0xd4283000, 0x80, 97, 99);
PXA168_DEVICE(ssp1, "pxa168-ssp", 0, SSP1, 0xd401b000, 0x40, 52, 53);
PXA168_DEVICE(ssp2, "pxa168-ssp", 1, SSP2, 0xd401c000, 0x40, 54, 55);
PXA168_DEVICE(ssp3, "pxa168-ssp", 2, SSP3, 0xd401f000, 0x40, 56, 57);
PXA168_DEVICE(ssp4, "pxa168-ssp", 3, SSP4, 0xd4020000, 0x40, 58, 59);
PXA168_DEVICE(ssp5, "pxa168-ssp", 4, SSP5, 0xd4021000, 0x40, 60, 61);
PXA168_DEVICE(fb, "pxa168-fb", -1, LCD, 0xd420b000, 0x1c8);
PXA168_DEVICE(keypad, "pxa27x-keypad", -1, KEYPAD, 0xd4012000, 0x4c);
PXA168_DEVICE(eth, "pxa168-eth", -1, MFU, 0xc0800000, 0x0fff);
PXA168_DEVICE(sdh1, "sdhci-pxav2", 0, SDH1, 0xd4280000, 0x100);
PXA168_DEVICE(sdh2, "sdhci-pxav2", 1, SDH1, 0xd4281000, 0x100);
PXA168_DEVICE(sdh3, "sdhci-pxav2", 2, SDH2, 0xd427e000, 0x100);
PXA168_DEVICE(sdh4, "sdhci-pxav2", 3, SDH2, 0xd427f000, 0x100);

struct resource pxa168_resource_gpio[] = {
	{
		.start	= 0xd4019000,
		.end	= 0xd4019fff,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= IRQ_PXA168_GPIOX,
		.end	= IRQ_PXA168_GPIOX,
		.name	= "gpio_mux",
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device pxa168_device_gpio = {
	.name		= "pxa-gpio",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(pxa168_resource_gpio),
	.resource	= pxa168_resource_gpio,
};

//////////////////Add usb host
struct resource pxa168_usb_host_resources[] = {  //bytim-tian
	/* USB Host conroller register base */
	[0] = {
		.start	= 0xd4209000 + 0x100,
		.end	= 0xd4209000 + 0x200,
		.flags	= IORESOURCE_MEM,
		.name	= "capregs",
	},
	/* USB PHY register base */
	[1] = {
		.start	= 0xd4206000,
		.end	= 0xd4206000 + 0xff,
		.flags	= IORESOURCE_MEM,
		.name	= "phyregs",
	},
	[2] = {
		.start	= IRQ_PXA168_USB2,
		.end	= IRQ_PXA168_USB2,
		.flags	= IORESOURCE_IRQ,
	},
};

static u64 pxa168_usb_host_dmamask = DMA_BIT_MASK(32);
struct platform_device pxa168_device_usb_host = { //bytim-tian
	//.name = "pxa168-ehci",
	.name = "pxa-sph",  //bytian
	.id   = 1,
	.dev  = {
		.dma_mask = &pxa168_usb_host_dmamask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},

	.num_resources = ARRAY_SIZE(pxa168_usb_host_resources),
	.resource      = pxa168_usb_host_resources,
};

int __init pxa168_add_usb_host(struct mv_usb_platform_data *pdata)
{
	pxa168_device_usb_host.dev.platform_data = pdata;
	return platform_device_register(&pxa168_device_usb_host);
}

struct resource pxa168_usb_otg_resources[] = {  //bytim-tian
	/* USB Host conroller register base */
	[0] = {
		.start	= 0xd4208000 + 0x100,
		.end	= 0xd4208000 + 0x200,
		.flags	= IORESOURCE_MEM,
		.name	= "capregs",
	},
	/* USB PHY register base */
	[1] = {
		.start	= 0xd4207000,
		.end	= 0xd4207000 + 0xff,
		.flags	= IORESOURCE_MEM,
		.name	= "phyregs",
	},
	[2] = {
		.start	= IRQ_PXA168_USB1,
		.end	= IRQ_PXA168_USB1,
		.flags	= IORESOURCE_IRQ,
	},
};
struct platform_device pxa168_device_usb_otg = {  //bytim-tian
	.name = "mv-otg",
	.id   = -1,
	.dev  = {
		.dma_mask = &pxa168_usb_host_dmamask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},

	.num_resources = ARRAY_SIZE(pxa168_usb_otg_resources),
	.resource      = pxa168_usb_otg_resources,
};
struct resource pxa168_usb_otg_ehci_resources[] = {  //bytim-tian
	/* USB Host conroller register base */
	[0] = {
		.start	= 0xd4208000 + 0x100,
		.end	= 0xd4208000 + 0x200,
		.flags	= IORESOURCE_MEM,
		.name	= "capregs",
	},
	/* USB PHY register base */
	[1] = {
		.start	= 0xd4207000,
		.end	= 0xd4207000 + 0xff,
		.flags	= IORESOURCE_MEM,
		.name	= "phyregs",
	},
	[2] = {
		.start	= IRQ_PXA168_USB1,
		.end	= IRQ_PXA168_USB1,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device pxa168_device_usb_otg_ehci = {  //bytim-tian
	.name = "pxa-sph",
	.id   = 0,
	.dev  = {
		.dma_mask = &pxa168_usb_host_dmamask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},

	.num_resources = ARRAY_SIZE(pxa168_usb_otg_resources),
	.resource      = pxa168_usb_otg_ehci_resources,
};
struct resource pxa168_usb_otg_gadget_resources[] = {  //bytim-tian
	/* USB Host conroller register base */
	[0] = {
		.start	= 0xd4208000 + 0x100,
		.end	= 0xd4208000 + 0x200,
		.flags	= IORESOURCE_MEM,
		.name	= "capregs",
	},
	/* USB PHY register base */
	[1] = {
		.start	= 0xd4207000,
		.end	= 0xd4207000 + 0xff,
		.flags	= IORESOURCE_MEM,
		.name	= "phyregs",
	},
	[2] = {
		.start	= IRQ_PXA168_USB1,
		.end	= IRQ_PXA168_USB1,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device pxa168_device_usb_otg_gadget = {  //bytim-tian
	.name = "mv-udc",
	.id   = -1,
	.dev  = {
		.dma_mask = &pxa168_usb_host_dmamask,
		.coherent_dma_mask = DMA_BIT_MASK(32),
	},

	.num_resources = ARRAY_SIZE(pxa168_usb_otg_resources),
	.resource      = pxa168_usb_otg_gadget_resources,
};
int __init pxa168_add_usb_otg(struct mv_usb_platform_data *pdata)
{
	pxa168_device_usb_otg.dev.platform_data = pdata;
	return platform_device_register(&pxa168_device_usb_otg);
}
int __init pxa168_add_usb_otg_ehci(struct mv_usb_platform_data *pdata)
{
	pxa168_device_usb_otg_ehci.dev.platform_data = pdata;
	return platform_device_register(&pxa168_device_usb_otg_ehci);
}
int __init pxa168_add_usb_otg_gadget(struct mv_usb_platform_data *pdata)
{
	pxa168_device_usb_otg_gadget.dev.platform_data = pdata;
	return platform_device_register(&pxa168_device_usb_otg_gadget);
}


/////////////// Add USB 2.0 OTG controller  */
unsigned u2o_get(unsigned base, unsigned offset)
{
	return readl(base + offset);
}
	
void u2o_set(unsigned base, unsigned offset, unsigned value)
{
	volatile unsigned int reg;
	
	reg = readl(base + offset);
	reg |= value;
	writel(reg, base + offset);
	__raw_readl(base + offset);
	
}
	
void u2o_clear(unsigned base, unsigned offset, unsigned value)
{
	volatile unsigned int reg;
	
	reg = readl(base + offset);
	reg &= ~value;
	writel(reg, base + offset);
	__raw_readl(base + offset);
}
	
void u2o_write(unsigned base, unsigned offset, unsigned value)
{
	writel(value, base + offset);
	__raw_readl(base + offset);
	
}



int pxa168_usb_phy_init(unsigned base)
{	
	static int init_done;
	int count;
	if (init_done) {
		printk(KERN_DEBUG "re-init phy\n\n");
		/* return; */
	}
#if 0  //bytian
	/* enable the pull up */
	if (cpu_is_pxa910_z0()) {
		if (cpu_is_pxa910()) {
			u32 U2H_UTMI_CTRL = 
				(u32)ioremap_nocache(0xc0000004, 4);
			writel(1<<20, U2H_UTMI_CTRL);
		}
	}

	/* Initialize the USB PHY power */
	if (cpu_is_pxa910()) {
		u2o_set(base, UTMI_CTRL, (1<<UTMI_CTRL_INPKT_DELAY_SOF_SHIFT)
			| (1<<UTMI_CTRL_PU_REF_SHIFT));
	}
#endif
	u2o_set(base, UTMI_CTRL, 1<<UTMI_CTRL_PLL_PWR_UP_SHIFT);
	u2o_set(base, UTMI_CTRL, 1<<UTMI_CTRL_PWR_UP_SHIFT);
	/* UTMI_PLL settings */
	u2o_clear(base, UTMI_PLL, UTMI_PLL_PLLVDD18_MASK
		| UTMI_PLL_PLLVDD12_MASK | UTMI_PLL_PLLCALI12_MASK
		| UTMI_PLL_FBDIV_MASK | UTMI_PLL_REFDIV_MASK
		| UTMI_PLL_ICP_MASK | UTMI_PLL_KVCO_MASK);

	u2o_set(base, UTMI_PLL, 0xee<<UTMI_PLL_FBDIV_SHIFT
		| 0xb<<UTMI_PLL_REFDIV_SHIFT | 3<<UTMI_PLL_PLLVDD18_SHIFT
		| 3<<UTMI_PLL_PLLVDD12_SHIFT | 3<<UTMI_PLL_PLLCALI12_SHIFT
		| 2<<UTMI_PLL_ICP_SHIFT | 3<<UTMI_PLL_KVCO_SHIFT);

	/* UTMI_TX */
	u2o_clear(base, UTMI_TX, UTMI_TX_TXVDD12_MASK
		| UTMI_TX_CK60_PHSEL_MASK | UTMI_TX_IMPCAL_VTH_MASK);
	u2o_set(base, UTMI_TX, 3<<UTMI_TX_TXVDD12_SHIFT
		| 4<<UTMI_TX_CK60_PHSEL_SHIFT | 5<<UTMI_TX_IMPCAL_VTH_SHIFT);

	/* UTMI_RX */
	u2o_clear(base, UTMI_RX, UTMI_RX_SQ_THRESH_MASK
		| UTMI_REG_SQ_LENGTH_MASK);
	if (cpu_is_pxa168())
		u2o_set(base, UTMI_RX, 7<<UTMI_RX_SQ_THRESH_SHIFT
			| 2<<UTMI_REG_SQ_LENGTH_SHIFT);
	else
		u2o_set(base, UTMI_RX, 0xa<<UTMI_RX_SQ_THRESH_SHIFT
			| 2<<UTMI_REG_SQ_LENGTH_SHIFT);

	/* UTMI_IVREF */
	if (cpu_is_pxa168())
		/* fixing Microsoft Altair board interface with NEC hub issue -
		 * Set UTMI_IVREF from 0x4a3 to 0x4bf */
		u2o_write(base, UTMI_IVREF, 0x4bf);

	/* calibrate */
	count = 10000;
	while(((u2o_get(base, UTMI_PLL) & PLL_READY)==0) && count--);
	if (count <= 0) printk("%s %d: calibrate timeout, UTMI_PLL %x\n", 
		__func__, __LINE__, u2o_get(base, UTMI_PLL));

	/* toggle VCOCAL_START bit of UTMI_PLL */
	udelay(200);
	u2o_set(base, UTMI_PLL, VCOCAL_START);
	udelay(40);
	u2o_clear(base, UTMI_PLL, VCOCAL_START);

	/* toggle REG_RCAL_START bit of UTMI_TX */
	udelay(200);
	u2o_set(base, UTMI_TX, REG_RCAL_START);
	udelay(40);
	u2o_clear(base, UTMI_TX, REG_RCAL_START);
	udelay(200);

	/* make sure phy is ready */
	count = 1000;
	while(((u2o_get(base, UTMI_PLL) & PLL_READY)==0) && count--);
	if (count <= 0) printk("%s %d: calibrate timeout, UTMI_PLL %x\n", 
		__func__, __LINE__, u2o_get(base, UTMI_PLL));

	if (cpu_is_pxa168()) {
		u2o_set(base, UTMI_RESERVE, 1<<5);
		u2o_write(base, UTMI_OTG_ADDON, 1);  /* Turn on UTMI PHY OTG extension */
	}

	init_done = 1;
	return 0;
}
int pxa168_usb_phy_deinit(unsigned base)
{
	if (cpu_is_pxa168())
		u2o_clear(base, UTMI_OTG_ADDON, UTMI_OTG_ADDON_OTG_ON);

	u2o_clear(base, UTMI_CTRL, UTMI_CTRL_RXBUF_PDWN);
	u2o_clear(base, UTMI_CTRL, UTMI_CTRL_TXBUF_PDWN);
	u2o_clear(base, UTMI_CTRL, UTMI_CTRL_USB_CLK_EN);
	u2o_clear(base, UTMI_CTRL, 1<<UTMI_CTRL_PWR_UP_SHIFT);
	u2o_clear(base, UTMI_CTRL, 1<<UTMI_CTRL_PLL_PWR_UP_SHIFT);
	return 0;
}

/////////////////// Add RTC
static struct resource pxa910_resource_rtc[] = {
	[0] = {
		.start  = 0xd4010000,
		.end    = 0xD40100ff,
		.flags  = IORESOURCE_MEM,
	},
	[1] = {
		.start  = IRQ_PXA168_RTC_INT,
		.end    = IRQ_PXA168_RTC_INT,
		.flags  = IORESOURCE_IRQ,
		.name   = "RTC_1HZ",
	},

	[2] = {
		.start  = IRQ_PXA168_RTC_ALARM,
		.end    = IRQ_PXA168_RTC_ALARM,
		.flags  = IORESOURCE_IRQ,
		.name   = "RTC_ALARM",
	},

};

struct platform_device pxa910_device_rtc = {
       .name           = "mmp-rtc",
       .id             = -1,
       .resource       = pxa910_resource_rtc,
       .num_resources  = ARRAY_SIZE(pxa910_resource_rtc),
};

int pxa168_add_rtc(void *data)
{
	pxa910_device_rtc.dev.platform_data = data;
	return platform_device_register(&pxa910_device_rtc);
}

void pxa168_restart(char mode, const char *cmd)
{
	soft_restart(0xffff0000);
}
