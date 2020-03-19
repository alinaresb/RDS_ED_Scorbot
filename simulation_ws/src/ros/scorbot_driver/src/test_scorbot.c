/** \file
 * \brief Example code for Simple Open EtherCAT master
 *
 * Usage : simple_test [ifname1]
 * ifname is NIC interface, f.e. eth0
 *
 * This is a minimal test.
 *
 * (c)Arthur Ketels 2010 - 2011
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "ethercat.h"

#define EC_TIMEOUTMON 500

char IOmap[4096];
OSAL_THREAD_HANDLE thread1;
int expectedWKC;
boolean needlf;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;

typedef struct PACKED {
  uint16_t controlRegA;
  uint16_t controlRegb;
  int16_t  currentRef;
  int16_t currentLim;
  uint16_t pidCurrentKp;
  uint16_t pidCurrentKi;
  uint16_t pidCurrentKd;
} setpoint_t;

typedef struct PACKED {
  int16_t encPosition;
  int16_t encSpeed;
  int16_t current;
  uint16_t limits;
} joint_data_t;

int16_t currentRef = 0;

void simpletest(char *ifname)
{
    int i, j, oloop, iloop, chk;
    needlf = FALSE;
    inOP = FALSE;

    uint16_t brightness = 1000U;

   printf("Starting simple test\n");

   /* initialise SOEM, bind socket to ifname */
   if (ec_init(ifname))
   {
      printf("ec_init on %s succeeded.\n",ifname);
      /* find and auto-config slaves */


      if ( ec_config_init(FALSE) > 0 )
      {
        printf("%d slaves found and configured.\n", ec_slavecount);

        ec_config_map(&IOmap);

        ec_configdc();

        printf("Slaves mapped, state to SAFE_OP.\n");
        /* wait for all slaves to reach SAFE_OP state */
        ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE * 4);


        oloop = ec_slave[1].Obytes;
        printf("Output bytes: %d\n", oloop);
        if ((oloop == 0) && (ec_slave[1].Obits > 0)) oloop = 1;
        //if (oloop > 10) oloop = 10;
        iloop = ec_slave[1].Ibytes;
        printf("Input bytes: %d\n", iloop);
        if ((iloop == 0) && (ec_slave[1].Ibits > 0)) iloop = 1;
        //if (iloop > 10) iloop = 10;

        printf("Segments : %d : %d %d %d %d\n",ec_group[0].nsegments ,ec_group[0].IOsegment[0],ec_group[0].IOsegment[1],ec_group[0].IOsegment[2],ec_group[0].IOsegment[3]);

        /* Request for OPERATIONAL */
        printf("Request operational state for all slaves\n");
        expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
        printf("Calculated workcounter %d\n", expectedWKC);
        ec_slave[0].state = EC_STATE_OPERATIONAL;
        /* send one valid process data to make outputs in slaves happy*/
        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);
        /* request OP state for all slaves */
        ec_writestate(0);
        chk = 40;
        /* wait for all slaves to reach OP state */
        do
        {
          ec_send_processdata();
          ec_receive_processdata(EC_TIMEOUTRET);
          ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
        }
        while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));
        if (ec_slave[0].state == EC_STATE_OPERATIONAL )
        {
          printf("Operational state reached for all slaves.\n");
          inOP = TRUE;
          int prescaler = 0;
              /* cyclic loop */
          for(i = 1; i <= 2000; i++)
          {
            ec_send_processdata();
            wkc = ec_receive_processdata(EC_TIMEOUTRET);

            if(wkc >= expectedWKC)
            {
/*              printf("Processdata cycle %4d, WKC %d , O:", i, wkc);

              for(j = 0 ; j < oloop; j++)
              {
                  printf(" %2.2x", *(ec_slave[1].outputs + j));
              }

              printf(" I:");
              for(j = 0 ; j < iloop; j++)
              {
                  printf(" %2.2x", *(ec_slave[1].inputs + j));
              }*/
              joint_data_t data = *((joint_data_t*) (ec_slave[1].inputs));

              /*printf(" Lim %d ", data.limits);*/

              setpoint_t* data_out = ((setpoint_t*) (ec_slave[1].outputs));
              data_out->currentRef = currentRef;
              prescaler = prescaler > 100 ? 0 : prescaler+1;
              if (prescaler==100)
                currentRef = currentRef == 0 ? 32000 : 0;

              /*printf(" T:%"PRId64"\r",ec_DCtime);*/
              needlf = TRUE;
            }
            osal_usleep(1000);
          }
          inOP = FALSE;
        }
        else
        {
          printf("Not all slaves reached operational state.\n");
          ec_readstate();
          for(i = 1; i<=ec_slavecount ; i++)
          {
            if(ec_slave[i].state != EC_STATE_OPERATIONAL)
            {
                printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                  i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
            }
          }
        }

        printf("\nRequest init state for all slaves\n");
        ec_slave[0].state = EC_STATE_INIT;
        /* request INIT state for all slaves */
        ec_writestate(0);

        ec_slave[0].state = EC_STATE_INIT;
        /* request INIT state for all slaves */
        ec_writestate(0);
        chk = 40;
        /* wait for all slaves to reach OP state */
        do
        {
          ec_send_processdata();
          ec_receive_processdata(EC_TIMEOUTRET);
          ec_statecheck(0, EC_STATE_INIT, 50000);
        }
        while (chk-- && (ec_slave[0].state != EC_STATE_INIT));
        if (ec_slave[0].state == EC_STATE_INIT )
        {
          printf("\nRequest init state for all slaves [OK]\n");
        }
        else
        {
          printf("Not all slaves reached init state. [FAIL]\n");
        }

      }
      else
      {
          printf("No slaves found!\n");
      }
      printf("End simple test, close socket\n");
      /* stop SOEM, close socket */
      ec_close();
    }
    else
    {
        printf("No socket connection on %s\nExcecute as root\n",ifname);
    }
}

OSAL_THREAD_FUNC ecatcheck( void *ptr )
{
    int slave;

    while(1)
    {
        if( inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate))
        {
            if (needlf)
            {
               needlf = FALSE;
               printf("\n");
            }
            /* one ore more slaves are not responding */
            ec_group[currentgroup].docheckstate = FALSE;
            ec_readstate();
            for (slave = 1; slave <= ec_slavecount; slave++)
            {
               if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
               {
                  ec_group[currentgroup].docheckstate = TRUE;
                  if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
                  {
                     printf("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
                     ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                     ec_writestate(slave);
                  }
                  else if(ec_slave[slave].state == EC_STATE_SAFE_OP)
                  {
                     printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
                     ec_slave[slave].state = EC_STATE_OPERATIONAL;
                     ec_writestate(slave);
                  }
                  else if(ec_slave[slave].state > 0)
                  {
                     if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
                     {
                        ec_slave[slave].islost = FALSE;
                        printf("MESSAGE : slave %d reconfigured\n",slave);
                     }
                  }
                  else if(!ec_slave[slave].islost)
                  {
                     /* re-check state */
                     ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                     if (!ec_slave[slave].state)
                     {
                        ec_slave[slave].islost = TRUE;
                        printf("ERROR : slave %d lost\n",slave);
                     }
                  }
               }
               if (ec_slave[slave].islost)
               {
                  if(!ec_slave[slave].state)
                  {
                     if (ec_recover_slave(slave, EC_TIMEOUTMON))
                     {
                        ec_slave[slave].islost = FALSE;
                        printf("MESSAGE : slave %d recovered\n",slave);
                     }
                  }
                  else
                  {
                    ec_slave[slave].islost = FALSE;
                    printf("MESSAGE : slave %d found\n",slave);
                  }
               }
            }
            if(!ec_group[currentgroup].docheckstate)
              printf("OK : all slaves resumed OPERATIONAL.\n");
        }
        osal_usleep(10000);
    }
}

int main(int argc, char *argv[])
{
  printf("SOEM (Simple Open EtherCAT Master)\nSimple test\n");

  if (argc > 1)
  {
    /* create thread to handle slave error handling in OP */
    osal_thread_create(&thread1, 128000, &ecatcheck, (void*) &ctime);
    /* start cyclic part */
    simpletest(argv[1]);
  }
  else
  {
    printf("Usage: simple_test ifname1\nifname = eth0 for example\n");
  }

  printf("End program\n");
  return (0);
}