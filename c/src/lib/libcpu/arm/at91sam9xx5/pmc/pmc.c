/*
at91sam9xx5
baconxu@gmail.com
 */

#include <rtems.h>
#include <bsp.h>
#include <at91sam9xx5.h>

int at91sam9xx5_get_mainclk(void)
{
    return BSP_MAIN_FREQ;
}

int at91sam9xx5_get_slck(void)
{
    return BSP_SLCK_FREQ;
}

int at91sam9xx5_get_mck(void)
{
    uint32_t mck_reg;
    uint32_t mck_freq = 0;  /* to avoid warnings */
    uint32_t pll_reg;
    int prescaler = 0;  /* to avoid warnings */

    //mck_reg = PMC_REG(PMC_MCKR);
    mck_reg = REG_PMC_MCKR;

	switch(mck_reg & PMC_MCKR_PRES_Msk) {
	case PMC_MCKR_PRES_CLOCK:
		prescaler = 1;
		break;
	case PMC_MCKR_PRES_CLOCK_DIV2:
		prescaler = 2;
		break;
	case PMC_MCKR_PRES_CLOCK_DIV4:
		prescaler = 4;
		break;
	case PMC_MCKR_PRES_CLOCK_DIV8:
		prescaler = 8;
		break;
	case PMC_MCKR_PRES_CLOCK_DIV16:
		prescaler = 16;
		break;
	case PMC_MCKR_PRES_CLOCK_DIV32:
		prescaler = 32;
		break;
	case PMC_MCKR_PRES_CLOCK_DIV64:
		prescaler = 64;
		break;
	default:
		//it's strange, datasheet said, 
		//7 stands for clock that divided by 3.
		//but the official file header haven't defined it.
		//baconxu@gmail.com 
		prescaler = 3;
		break;
	}

	/* Let's find out what MCK's source is */
	switch (mck_reg & PMC_MCKR_CSS_Msk) {
	case PMC_MCKR_CSS_SLOW_CLK:
		/* I'm assuming the slow clock is 32.768 MHz */
		mck_freq = at91sam9xx5_get_slck() / prescaler;
		break;

	case PMC_MCKR_CSS_MAIN_CLK:
		mck_freq = at91sam9xx5_get_mainclk() / prescaler;
		break;

	case PMC_MCKR_CSS_PLLA_CLK:
		pll_reg = REG_CKGR_PLLAR;
		mck_freq = at91sam9xx5_get_mainclk() / prescaler;
		mck_freq = mck_freq / (pll_reg & CKGR_PLLAR_DIVA_Msk);
		mck_freq = mck_freq * (((pll_reg & CKGR_PLLAR_MULA_Msk) >> CKGR_PLLAR_MULA_Pos) + 1);
		if (mck_reg & PMC_MCKR_PLLADIV2)
		{
			mck_freq = mck_freq / 2;
		}
		break;

	case PMC_MCKR_CSS_UPLL_CLK:
		/*I don't know how to deal with it. just same as PMC_MCKR_CSS_MAIN_CLK.
		baconxu@gmail.com
		*/
		mck_freq = at91sam9xx5_get_mainclk() / prescaler;
    #if 0
		pll_reg = PMC_REG(PMC_PLLBR);
		mck_freq = at91sam9g25_get_mainclk() / prescaler;
		mck_freq = mck_freq / (pll_reg & PMC_PLLBR_DIV_MASK);
		mck_freq = mck_freq * (((pll_reg & PMC_PLLBR_MUL_MASK) >> 16) + 1);
    #endif
		break;
	}

	if ((mck_reg & PMC_MCKR_MDIV_Msk) == PMC_MCKR_MDIV_PCK_DIV2) {
		mck_freq = mck_freq / 2;
	} else if ((mck_reg & PMC_MCKR_MDIV_Msk) == PMC_MCKR_MDIV_PCK_DIV3) {
		mck_freq = mck_freq / 3;
	} else if ((mck_reg & PMC_MCKR_MDIV_Msk) == PMC_MCKR_MDIV_PCK_DIV4) {
		mck_freq = mck_freq / 4;
	}


	return mck_freq;
}

