/*******************************************************************************
* Function Name: main
********************************************************************************
*
* Summary:
*  main() performs the following functions:
*  1. Initializes CapSense and SCB blocks
*  2. Scans & processes proximity sensor (for low power wake-on-approach) and keypad buttons.
*
* Date: 			26-Jan-2020
* Author:			David Waskevich (djjw)
* Project Name:	    TouchController 2020-001
*
* Description: Migration of keypad design from MBR3 to PSoC4 (CY8C4014LQI-422). Mimics EZ-Click setup.
*              Using CySysPmDeepSleep() to put system in deep sleep. Wakeup source is WDT.
*              WDT/deep-sleep interval set with LOOK_FOR_PROX_MSEC macro (initially set to 80 msec).
*              Active scanning interval set with LOOK_FOR_TOUCH_MSEC macro (initially set to 20 msec).
*              ILO compensation is applied to WDT match values to improve accuracy (+/-2% IMO).
*
* Hardware - CY8CKIT-040 Pioneer Kit (CY8C4014LQI-422)
*
* Requirements - put system/CPU to deep sleep for LOOK_FOR_PROX msec to save power. Wake up system with
*                deep sleep WDT interrupt. Scan buttons after wakeup. Return to deep sleep after inactivity.
*
* Documents referenced: System Reference Guide (accessible from inside Creator)
*                       '-> PSoC 4 Low-Frequency Clock (cy_lfclk) v1.20
*                       CE210292 - ILO compensation
*
*  Update 2-18-2020 [djjw] ... switching hardware to QFN24 PCB (2020-001)
*
*  Update 3-5-2020 [djjw]  ... modified main loop to scan only Proximity widget until proximity
*                              event occurs, then switch to scanning all widgets.
*                              uCurrent Gold oscilloscope measurements confirm prox scan time and
*                              total scan time for all widgets.
*
*  Update 3-8-2020 [djjw]  ... changed to SmartSense auto-tuning. Using CY3280-SLM for sensors (only 5 buttons).
*                              Rigol DM3058E - 1.275 mA constant key press with LED, 368 uA in
*                              look-for-touch. Look-for-Prox ... 2.9 uA deep sleep, 54 uA MAX,
*                              27 uA average. (Note - powered by 3x AAA battery pack ... 4.5V)
*
*  Update 10-24-2021 [djjw] ... switching hardware to reflow oven board.
*
*
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
* Cypress reserves the right to make changes to the Software without notice. 
* Cypress does not assume any liability arising out of the application or use 
* of the Software or any product or circuit described in the Software. Cypress
* does not authorize its products for use in any products where a malfunction
* or failure of the Cypress product may reasonably be expected to result in 
* significant property damage, injury or death (“High Risk Product”). By 
* including Cypress’s product in a High Risk Product, the manufacturer of such  
* system or application assumes all risk of such use and in doing so agrees to  
* indemnify Cypress against all liability. 
*
*******************************************************************************/

/*******************************************************************************
*   Included Headers
*******************************************************************************/
#include "project.h"

/*******************************************************************************
*   Macros and #define Constants
*******************************************************************************/

#define PROX_ENABLE 1u

//#define TUNER_ENABLE 1u

/* NOTE - Cmod measurement and Sensor_Cp test requires CapSense BIST library. Turn off SmartSense and enable BIST. */
#if (CapSense_ENABLE == CapSense_SELF_TEST_EN)
    /* This is a bring-up test with its own forever loop. If BIST_ENABLE is enabled/defined, no other code will run. */
    #define BIST_ENABLE 1u
#endif

//#define REPORT_CALIBRATED_ILO_VALUES 1u
//#define REPORT_RAW_COUNTS 1u

#define FALSE 0u
#define TRUE 1u

/* Device power mode related macros. LOOK_FOR_PROX_MSEC saves power but slows
   down first-touch response time. 80 mSec = 12.5 Hz "look for prox" scan rate. */
#define LOOK_FOR_TOUCH_MSEC (20u)
#define LOOK_FOR_PROX_MSEC  (80u)

/* State timeout - this is the number of fast scan cycles elapsed before switching
   to slow scan when touch surface is inactive.  This value is calculated by
   (Desired Delay/LOOK_FOR_TOUCH_MSEC). For 6 second delay, the value is 300. */
#define STATE_TIMEOUT       (300u)

#define NO_EVENT 0x0000
#define PROX_MASK 0x8000

/* I2C commands ... arbitrary at this point */
#define ENTER_BOOTLOADER 0x01
#define SW_RESET 0x02
#define USER_COMMAND_1 0x03

/* Number of sensors to load in I2C register */
//#define NUM_SENSORS_TO_REPORT   (2u)

/*******************************************************************************
*   Function Prototypes
*******************************************************************************/
void WDT_Start(void);
CY_ISR(WDT_Interrupt);
uint32 calibrateILO(uint16 desiredDelay);

/*******************************************************************************
*   Globals
*******************************************************************************/
/* Variable that stores the watchdog overflow duration, will be initialized to calibrated watchdogFastScan. */
uint32 watchdogMatchValue;

/* Fast scan delay counter */
uint16 fastScanDelayCounter = STATE_TIMEOUT;

/* define register map that will be exposed to I2C host */
struct RegisterMap {
	uint8 CommandRegister; /* I2C master can write to this register to initiate slave-side commands. */
	uint8 FirmwareVersion; /* current version of firmware */
    uint16 ScanResult; /* holds scan results */
    #if (REPORT_CALIBRATED_ILO_VALUES || BIST_ENABLE)
        uint16 CalibratedScanCounts[2]; /* temporary ... remove after testing */
    #endif
    #if (REPORT_RAW_COUNTS || BIST_ENABLE)
    	uint16 RawCountData[CapSense_TOTAL_CSD_SENSORS]; /* holds sensor raw counts - LSByte first */
    #endif
} RegisterMap;

int main ()
{    
    Pin_INT_REQ_Write(0);
    
//    while(1)
//    {
//        
//        Pin_LED_Write(~Pin_LED_Read());
//        Pin_INT_REQ_Write(~Pin_INT_REQ_Read());
//        CyDelay(500);
//        
//    }
    
    uint16 previousValue = NO_EVENT; /* initialize previous value to NO_EVENT */
	uint8 i, interruptState, statusMasterRead2, scanAllWidgets; /* miscellaneous local variables */
	uint32 status;
    
    #ifdef REPORT_RAW_COUNTS
        uint8 j, loopCount;
    	CapSense_FLASH_WD_STRUCT const *ptrFlashWdgt; /* pointer variable to point to widget array in Flash */
    	CapSense_RAM_SNS_STRUCT *ptrSns; /* pointer variable to point to widget's sensor array in SRAM */
	#endif
    
    scanAllWidgets = FALSE; /* initialize flag */
	statusMasterRead2 = FALSE; /* initialize flag */
    uint16 watchdogFastScan, watchdogSlowScan; /* WDT match count values for Active and DeepSleep scan cycles. */
	
	RegisterMap.CommandRegister = 0x00;
	RegisterMap.FirmwareVersion = 0xAA; /* current version of the FW */
	RegisterMap.ScanResult = NO_EVENT; /* initialize value to NO_EVENT */
    
    /* djjw - calibrate WDT/ILO match counts for fast and slow scans against IMO to achieve better (+/- 2.5%) accuracy */
    watchdogFastScan = (uint16) calibrateILO(LOOK_FOR_TOUCH_MSEC);
    watchdogSlowScan = (uint16) calibrateILO(LOOK_FOR_PROX_MSEC);
    #ifdef REPORT_CALIBRATED_ILO_VALUES
        RegisterMap.CalibratedScanCounts[0] = watchdogFastScan;
        RegisterMap.CalibratedScanCounts[1] = watchdogSlowScan;
    #endif
    watchdogMatchValue = watchdogFastScan; /* set initial WDT match value to fast scan rate */
    
	/* Enable Global interrupts */
    CyGlobalIntEnable;
    
    /* Start EZI2C block */
    EZI2C_Start();
	
	/* djjw - adding tuner */
	#ifdef TUNER_ENABLE
	
		/* Set up communication data buffer to CapSense data structure to 
	     * expose to I2C master at primary slave address request        
	     */
	    EZI2C_EzI2CSetBuffer1(sizeof(CapSense_dsRam), sizeof(CapSense_dsRam),\
	                         (uint8 *)&CapSense_dsRam);
	
	#endif
    
    /* Set up communication data buffer to be exposed to I2C master at secondary slave address. */
	EZI2C_EzI2CSetBuffer2(sizeof(RegisterMap), sizeof(RegisterMap.CommandRegister), (uint8 *) &RegisterMap);
	
    /* Start the Watchdog Timer */
    WDT_Start();
    
	#ifdef BIST_ENABLE
        /* djjw - adding BIST checks (measure/validate Cmod and  */    
        uint16 capacitanceValue;
        capacitanceValue = (uint16) CapSense_GetExtCapCapacitance(CapSense_TST_CMOD_ID);
        RegisterMap.CalibratedScanCounts[0] = capacitanceValue;
	
        for(;;)
        {
            uint16 capacitanceValue;
            capacitanceValue = (uint16) CapSense_GetExtCapCapacitance(CapSense_TST_CMOD_ID);
            RegisterMap.CalibratedScanCounts[0] = capacitanceValue;
            
            for(i = 0; i < CapSense_TOTAL_CSD_SENSORS; i++)
            {
                capacitanceValue = (uint16) CapSense_GetSensorCapacitance(i, 0);
                RegisterMap.RawCountData[i] = capacitanceValue;
            }
            
            RegisterMap.ScanResult += 1;
            
            do
            {
                status = EZI2C_EzI2CGetActivity(); /* Get slave status. */
                
            } while(0 == (status & EZI2C_EZI2C_STATUS_READ2));
        }
        
	#endif    
	
    /* Start CapSense block - Initializes CapSense Data structure and performs first scan to set up sensor baselines. */
    CapSense_Start();
    
    for(;;)
    {
        if(TRUE == scanAllWidgets)
        {
            /* Scan all widgets */
    	    CapSense_ScanAllWidgets();
        }
        else
        {
            /* Scan proximity widget only */
            CapSense_SetupWidget(CapSense_PROXIMITY0_WDGT_ID);
    	    CapSense_Scan();
        }
        
        Pin_INT_REQ_Write(0);
		
        /* Put the CPU to sleep. When CapSense hardware completes scanning, the interrupt will wake up the CPU. */
        CySysPmSleep();
        
        Pin_INT_REQ_Write(1);

        /* Make sure scan is complete ... not sure this is needed but it was in the code example. Maybe convert to if() in case I2C is the wakeup source??? */
        while(!(CapSense_NOT_BUSY == CapSense_IsBusy()))
			;
        
        #ifdef TUNER_ENABLE
            /* Update CapSense parameters set via CapSense tuner */
            CapSense_RunTuner();
        #endif
        
        if(TRUE == scanAllWidgets)
        {
            /* Process all widgets */
            CapSense_ProcessAllWidgets();
        }
        else
        {
            /* Process proximity widget only */
            CapSense_ProcessWidget(CapSense_PROXIMITY0_WDGT_ID);
        }
        
		/* Check if any widget is active. If so, set/keep the scan rate in the fast mode and set flag to scan all widgets. */
        if(CapSense_IsAnyWidgetActive() || CapSense_IsProximitySensorActive(CapSense_PROXIMITY0_WDGT_ID, CapSense_PROXIMITY0_SNS0_ID))
		{
			/* Reset fast scan delay counter to state timeout value */
            fastScanDelayCounter = STATE_TIMEOUT;
            
            /* Set Watchdog interrupt rate for fast scan */
            watchdogMatchValue = watchdogFastScan;            
            scanAllWidgets = TRUE;			
			Pin_LED_Write(0); /* djjw - debug code ... remove later */
		}
		else /* If not, set scan rate to LOOK_FOR_PROX if idle for more than STATE_TIMEOUT scan cycles. */
		{
			/* If fast scan delay counter is not zero */
            if(fastScanDelayCounter != 0)
            {
                /* Decrement fast scan delay counter */
                fastScanDelayCounter--;
                
                /* If fast scan delay counter has become zero, then this means the touch
                   surface has not been active for a long time.  Switch to slow scan and reset flag to scan only prox. */
                if(fastScanDelayCounter == 0)
                {
                    watchdogMatchValue = watchdogSlowScan;
                    scanAllWidgets = FALSE;
                }
            }
			
			Pin_LED_Write(1); /* djjw - debug code ... remove later */
		}
			
		/* Check if I2C is busy (should only happen in factory read mode). Idle here if Master transaction is in progress. */
		/* djjw - changing this section to do{ ... } while() loop. Wait here for idle status of I2C bus. */
		do
		{
			/* Enter critical section to stop interrupts. */
			interruptState = CyEnterCriticalSection();
			
			status = EZI2C_EzI2CGetActivity(); /* Get slave status. */
			
			if(status & EZI2C_EZI2C_STATUS_READ2) /* Test to see if slave has been read by master. */
				statusMasterRead2 = TRUE; /* Save/store result by setting flag if Master has read Slave. Will be used and reset later. */
			
			/* Now check if slave is busy. */
			status &= EZI2C_EZI2C_STATUS_BUSY;
			if(0 != status) /* I2C bus is still busy. */
			{
				/* I2C transaction still in progress, exit Critical Section and remain in do{ ... } while() loop. */
				CyExitCriticalSection(interruptState);
			}
			else /* I2C bus is idle, ok to manipulate EZI2C registers. */
			{
                #ifdef PROX_ENABLE
					/* Process prox. */
                    if(CapSense_IsProximitySensorActive(CapSense_PROXIMITY0_WDGT_ID, CapSense_PROXIMITY0_SNS0_ID))
					{
						RegisterMap.ScanResult |= PROX_MASK;
					}
                    else
                        RegisterMap.ScanResult &= ~PROX_MASK;
				#endif
                
                /* Now process the buttons. */
				for( i = CapSense_BUTTON0_WDGT_ID; i <= CapSense_TOTAL_WIDGETS - CapSense_BUTTON0_WDGT_ID; i++ )
				{
					if(CapSense_IsWidgetActive(i))
					{
						RegisterMap.ScanResult |= 1 << (i - CapSense_BUTTON0_WDGT_ID);
					}
					else RegisterMap.ScanResult &= ~(1 << (i - CapSense_BUTTON0_WDGT_ID));
				}
				
                #ifdef REPORT_RAW_COUNTS
    				/* adding sensor raw counts to RegisterMap */
    				loopCount = 0;
    				for(i = 0; i < CapSense_TOTAL_CSD_SENSORS; i++)
    				{
    					/* using structure pointer to access widget array */
    					ptrFlashWdgt = &CapSense_dsFlash.wdgtArray[i]; /* use loop index "i" to point to current widget */
    					ptrSns = ptrFlashWdgt->ptr2SnsRam; /* set pointer to point to the current widget's sensor data in SRAM */
    					
    					for(j = 0; j < ptrFlashWdgt->totalNumSns; j++) /* process all sensors associated with current widget */
    					{
    						/* using structure pointer to access sensor raw counts */
    						RegisterMap.RawCountData[loopCount] = ptrSns->raw[0]; /* use Sensor pointer to retrieve raw count */
    						ptrSns++; /* move to next sensor in current widget */
    						loopCount++; /* increment index to next location in RegisterMap */
    					}
    				}
                #endif
				
				/* Exit critical section and restore interrupt state. */
				CyExitCriticalSection(interruptState);
			}
		} while (0 != status);

        /* Handle these all the time */
		
		/* Process host commands */
		switch( RegisterMap.CommandRegister )
		{
			case ENTER_BOOTLOADER:
				/* placeholder for remote bootload command from host */
				break;
			case SW_RESET:
				CySoftwareReset();
				break;
			case USER_COMMAND_1:
				Pin_LED_Write(0);
				CyDelay(200);
				Pin_LED_Write(1);
                RegisterMap.CommandRegister = 0x00;
				break;
			default:
				break;			
		}

		/* Signal Host if new touch data is detected. */
        if(RegisterMap.ScanResult != previousValue )
        {
            Pin_INT_REQ_Write(0); //signal to host
            
            CyDelay(250);
            Pin_INT_REQ_Write(1);
            
            
        }
        previousValue = RegisterMap.ScanResult;    //save history of scan result
		
		/* Check statusMasterRead1 flag to see if Master serviced INT and read Slave. */
		if(TRUE == statusMasterRead2)
        {
            Pin_INT_REQ_Write(1);  //host read me, reset the IRQ line  
			statusMasterRead2 = FALSE; /* reset flag */
        }
		
		/* Prepare system for deep sleep ... first, enter critical section to lock the slave state. */
		do
		{
			/* Enter critical section to stop interrupts. */
			interruptState = CyEnterCriticalSection();
			
			status = EZI2C_EzI2CGetActivity(); /* Get slave status. */
			
			if(status & EZI2C_EZI2C_STATUS_READ2) /* Test to see if slave has been read by master. */
				statusMasterRead2 = TRUE; /* Save/store result by setting flag if Master has read Slave. Will be reset later. */
			
			/* Now check if slave is busy. */
			status &= EZI2C_EZI2C_STATUS_BUSY;

			if(0 != status) /* I2C bus is still busy. */
			{
				/* I2C transaction still in progress, exit Critical Section and remain in do{ ... } while() loop. */
				CyExitCriticalSection(interruptState);
			}
			else /* I2C bus is idle. */
			{
				/* Slave is not busy, ok to enter Deep Sleep. */
				EZI2C_Sleep(); /* Configure the slave to be a wakeup source */
				
				CapSense_Sleep();
				
				/* Put CPU to deep sleep. Wakeup occurs on WDT timeout or Master I2C transaction. */
				CySysPmDeepSleep();
				
				/* Exit critical section to continue slave operation */
				CyExitCriticalSection(interruptState);
				
				CapSense_Wakeup();
                
				EZI2C_Wakeup(); /* Configure the slave to active mode operation */
			}
		} while (0 != status);
    }
}

/******************************************************************************
* Function Name: WDT_Interrupt
*******************************************************************************
*
* Summary:
*  Handles the Interrupt Service Routine for the WDT timer.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Theory: The interrupt is cleared on the ISR as watchdog in this project is 
*         used for timing maintenance only. Match value is updated to maintain 
*         the loop time. 
*
* Side Effects: None
*
* Note:
*
*******************************************************************************/
CY_ISR(WDT_Interrupt)
{
    /* Clear the watchdog interrupt */
    CySysWdtClearInterrupt();    
    
    /* WDT match value is updated in order to obtain periodic interrupts */
    CySysWdtWriteMatch(CySysWdtReadMatch() + watchdogMatchValue); 
}

/******************************************************************************
* Function Name: WDT_Start
*******************************************************************************
*
* Summary:
*  Configures WDT.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Theory: This API unmasks the WDT interrupt to route the interrupt to CPU and 
*         enables WDT.
*
* Side Effects: None
*
* Note:
*
*******************************************************************************/
void WDT_Start(void)
{
    /* Setup ISR */
    WDT_Interrupt_StartEx(WDT_Interrupt);
    
     /* WDT match value is updated in order to obtain periodic interrupts */
    CySysWdtWriteMatch(CySysWdtReadMatch() + watchdogMatchValue); 
    
    /* Enable WDT interrupt */
    CySysWdtUnmaskInterrupt();
    
    /* Start WDT */
    CySysWdtEnable();
}

/******************************************************************************
* Function Name: calibrateILO
*******************************************************************************
*
* Summary:
*  Calibrates/compensates WDT match counts against IMO to improve accuracy.
*
* Parameters:
*  Desired delay in msec.
*
* Return:
*  Calibrated/compensated match count value.
*
* Theory: This function makes use of CyLFClk API's. See Help|System Reference Guides.
*
* Side Effects: None
*
* Note:
*
*******************************************************************************/
uint32 calibrateILO(uint16 desiredDelay)
{
    /* CE210292 */
    #define MILISECONDS_TO_MICROSECONDS (1000u)    
    uint32 iloMatchCounts;
    
    CySysClkIloStartMeasurement(); /* Start ILO accuracy measurement against IMO (+/-2%) */
	
	/* Wait for compensation to complete */
	while(CYRET_SUCCESS != CySysClkIloCompensate((desiredDelay * MILISECONDS_TO_MICROSECONDS), &iloMatchCounts))
	;

    /* Once the counts are ready - stop the ILO measurement */
    CySysClkIloStopMeasurement();

    return iloMatchCounts;
}

/* [] END OF FILE */