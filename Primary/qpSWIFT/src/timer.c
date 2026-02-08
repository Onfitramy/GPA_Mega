#include "timer.h"


/*! For Windows machines */
#if (defined WIN32 || _WIN64)

void tic(qp_timer* t)
{
	QueryPerformanceFrequency(&t->freq);
	QueryPerformanceCounter(&t->tic);
}

qp_real toc(qp_timer* t)
{
	QueryPerformanceCounter(&t->toc);
	return ((t->toc.QuadPart - t->tic.QuadPart) / (qp_real)t->freq.QuadPart);
}

/*! For macOS */
#elif (defined __APPLE__)

void tic(qp_timer* t)
{
	t->tic = mach_absolute_time();
}

qp_real toc(qp_timer* t)
{

	uint64_t duration; 
	t->toc = mach_absolute_time();
	duration = t->toc - t->tic;

	mach_timebase_info(&(t->tinfo));
	duration *= t->tinfo.numer;
	duration /= t->tinfo.denom;

	return (qp_real)duration / 1000000000;
}



#else
/*! For Posix machines */

static inline void dwt_init(void)
{
    if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0)
    {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CYCCNT = 0;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }
}

void tic(qp_timer* t)
{
    dwt_init();
    t->tic = DWT->CYCCNT;
}

qp_real toc(qp_timer* t)
{
    uint32_t now = DWT->CYCCNT;
    uint32_t diff = now - t->tic;   // unsigned handles overflow correctly

    return ((qp_real)diff) / ((qp_real)SystemCoreClock);
}


#endif
/*! @file */