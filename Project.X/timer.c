
#include "timer.h"

int choose_prescaler(int ms, int* pr, int* tckps) {
    long ticks = (7372800L / 4L) / 1000L * ms;
    long ticks_no_ps = ticks;
    if (ticks <= 65535) {
        *tckps = 0;
        *pr = ticks;
        return 0;
    }

    // prescaler 1:8
    ticks = ticks_no_ps / 8;
    if (ticks <= 65535) {
        *tckps = 1;
        *pr = ticks;
        return 0;
    }

    // prescaler 1:64
    ticks = ticks_no_ps / 64;
    if (ticks <= 65535) {
        *tckps = 2;
        *pr = ticks;
        return 0;
    }

    ticks = ticks_no_ps / 256;
    if (ticks <= 65535) {
        *tckps = 3;
        *pr = ticks;
        return 0;
    }

    return 1;
}

int tmr_setup_period(int timer, int ms) {
    switch (timer) {
        case TIMER1:
        {
            int pr, tckps;
            choose_prescaler(ms, &pr, &tckps);
            PR1 = pr;
            T1CONbits.TCKPS = tckps;
            T1CONbits.TCS = 0;
            T1CONbits.TGATE = 0;
            TMR1 = 0;
            T1CONbits.TON = 1;

            break;
        }
        case TIMER2:
        {
            int pr, tckps;
            choose_prescaler(ms, &pr, &tckps);
            PR2 = pr;
            T2CONbits.TCKPS = tckps;
            T2CONbits.TCS = 0;
            T2CONbits.TGATE = 0;
            TMR2 = 0;
            T2CONbits.TON = 1;

            break;
        }
        case TIMER3:
        {
            int pr, tckps;
            choose_prescaler(ms, &pr, &tckps);
            PR2 = pr;
            T3CONbits.TCKPS = tckps;
            T3CONbits.TCS = 0;
            T3CONbits.TGATE = 0;
            TMR3 = 0;
            T3CONbits.TON = 1;

            break;
        }
    }


    return 0;
}

int tmr_wait_period(int timer) {
    switch (timer) {
        case TIMER1:
        {
            if (IFS0bits.T1IF == 1) {
                IFS0bits.T1IF = 0;
                return 1;
            }

            while (IFS0bits.T1IF == 0);
            IFS0bits.T1IF = 0;
            break;
        }
        case TIMER2:
        {
            if (IFS0bits.T2IF == 1) {
                IFS0bits.T2IF = 0;
                return 1;
            }

            while (IFS0bits.T2IF == 0);
            IFS0bits.T2IF = 0;
            break;
        }
        case TIMER3:
        {
            if (IFS0bits.T3IF == 1) {
                IFS0bits.T3IF = 0;
                return 1;
            }

            while (IFS0bits.T3IF == 0);
            IFS0bits.T3IF = 0;
            break;
        }
    }

    return 0;
}

int tmr_wait_ms(int timer, int ms) {
    switch (timer) {
        case TIMER1:
        {
            int pr, tckps;
            choose_prescaler(ms, &pr, &tckps);
            PR1 = pr;
            T1CONbits.TCKPS = tckps;
            T1CONbits.TCS = 0;
            T1CONbits.TGATE = 0;
            
            T1CONbits.TON = 0;
            IFS0bits.T1IF = 0;
            TMR1 = 0;
            T1CONbits.TON = 1;
            while (IFS0bits.T1IF == 0);
            IFS0bits.T1IF = 0;
            T1CONbits.TON = 0;
            break;
        }
        case TIMER2:
        {
            int pr, tckps;
            choose_prescaler(ms, &pr, &tckps);
            PR2 = pr;
            T2CONbits.TCKPS = tckps;
            T2CONbits.TCS = 0;
            T2CONbits.TGATE = 0;
            
            T2CONbits.TON = 0;
            IFS0bits.T2IF = 0;
            TMR2 = 0;
            T2CONbits.TON = 1;
            while (IFS0bits.T2IF == 0);
            IFS0bits.T2IF = 0;
            T2CONbits.TON = 0;
            break;
        }
        case TIMER3:
        {
            int pr, tckps;
            choose_prescaler(ms, &pr, &tckps);
            PR3 = pr;
            T3CONbits.TCKPS = tckps;
            T3CONbits.TCS = 0;
            T3CONbits.TGATE = 0;
            
            T3CONbits.TON = 0;
            IFS0bits.T3IF = 0;
            TMR3 = 0;
            T3CONbits.TON = 1;
            while (IFS0bits.T3IF == 0);
            IFS0bits.T3IF = 0;
            T3CONbits.TON = 0;
            break;
        }
    }
    
    return 0;
}
