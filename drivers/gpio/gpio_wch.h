
#ifndef DRIVER_GPIO_GPIO_WCH_H_
#define DRIVER_GPIO_GPIO_WCH_H_

#define REG_BASE	DT_REG_ADDR_BY_IDX(DT_PARENT(DT_NODELABEL(gpio0)), 0)

/* PB22, PB23 correspond to BIT(8) and BIT(9) respectively */

#define WCH_GPIO_INT_BIT(pin)	 (uint16_t)(1U << (pin > 15 ? pin - 14 : pin))

#define WCH_GPIO_INT_MODE_EDGE			1
#define WCH_GPIO_INT_MODE_LEVEL			0

#define R16_PX_INT_EN(port)			(REG_BASE + 0x90 + port*2)
#define R16_PX_INT_MODE(port)		(REG_BASE + 0x94 + port*2)
#define R16_PX_INT_IF(port)			(REG_BASE + 0x9C + port*2)

#define GPIO_DIR_OUT				1
#define GPIO_DIR_IN					0
#define GPIO_OUT_HIGH				1
#define GPIO_OUT_LOW				0
#define GPIO_INT_MODE_EDGE_UP		1
#define GPIO_INT_MODE_EDGE_DOWN		0
#define GPIO_INT_MODE_LEVEL_HIGH	1
#define GPIO_INT_MODE_LEVEL_LOW		0


#define R32_PX_DIR(port)			(REG_BASE + 0xA0 + port*0x20)
#define R32_PX_PIN(port)			(REG_BASE + 0xA4 + port*0x20)
#define R32_PX_OUT(port)			(REG_BASE + 0xA8 + port*0x20)
#define R32_PX_CLR(port)			(REG_BASE + 0xAC + port*0x20)
#define R32_PX_PU(port)				(REG_BASE + 0xB0 + port*0x20)
#define R32_PX_PD_DRV(port)			(REG_BASE + 0xB4 + port*0x20)

#endif	/* DRIVER_GPIO_GPIO_WCH_H_ */
