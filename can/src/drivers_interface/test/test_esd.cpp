#include <stdio.h>
#include <ntcan.h>
/*
* This example demonstrates how the NTCAN-API can be used to open a handle,
* set a baudrate and wait for reception of a CAN frame with an
* identifier that has been previously enabled for this handle.
* Finally all proper cleanup operations are performed
*/
int main()
{
    int net = 0;
    /* logical net number (here: 0) */
    uint32_t mode = 0;
    /* mode bits for canOpen */
    int32_t txqueuesize = 8;
    /* maximum number of messages to transmit */
    int32_t rxqueuesize = 8;
    /* maximum number of messages to receive */
    int32_t txtimeout = 100;
    /* timeout for transmit in ms */
    int32_t rxtimeout = 10000; /* timeout for receiving data in ms */
    NTCAN_HANDLE rxhandle;
    /* can handle returned by canOpen() */
    NTCAN_RESULT retvalue;
    /* return values of NTCAN API calls */
    uint32_t baud = 2;
    /* configured CAN baudrate (here: 500 kBit/s.) */
    CMSG cmsg[8];
    /* buffer for can messages */
    int i;
    /* loop counter */
    int32_t len;
    /* Buffer size in # of messages for canRead() */
    /* ############################################################### */
    retvalue = canOpen(net,
                       mode,
                       txqueuesize,
                       rxqueuesize,
                       txtimeout,
                       rxtimeout,
                       &rxhandle);
    if (retvalue != NTCAN_SUCCESS)
    {
        printf("canOpen() failed with error %d!\n", retvalue);
        return (-1);
    }
    printf("function canOpen() returned OK !\n");
    /* ############################### ################################ */
    retvalue = canSetBaudrate(rxhandle, baud);
    if (retvalue != 0)
    {
        printf("canSetBaudrate() failed with error %d!\n", retvalue);
        canClose(rxhandle);
        return (-1);
    }
    printf("function canSetBaudrate() returned OK !\n");
    /* ############################################################### */
    retvalue = canIdAdd(rxhandle, 0x7FFF); /* Enable CAN-ID 0 */
    if (retvalue != NTCAN_SUCCESS)
    {
        printf("canIdAdd() failed with error %d!\n", retvalue);
        canClose(rxhandle);
        return (-1);
    }
    printf("function canIdAdd() returned OK !\n");
    /* ############################################################### */
    do
    {
        /*
* Set max numbers of messages that can be returned with
* one canRead() call according to buffer size
*/
        len = 8;
        retvalue = canRead(rxhandle, &cmsg[0], &len, NULL);
        if (retvalue == NTCAN_RX_TIMEOUT)
        {
            printf("canRead() returned timeout\n");
            continue;
        }
        else if (retvalue != NTCAN_SUCCESS)
        {
            printf("canRead() failed with error %d!\n", retvalue);
        }
        else
        {
            printf("function canRead() returned OK !\n");
            printf("Id of received message :%x!\n", cmsg[0].id);
            printf("Len of received message :%x!\n", (cmsg[0].len & 0x0f));
            printf("Rtr of received message :%x!\n", ((cmsg[0].len & 0x10) >> 4));
            for (i = 0; i < (cmsg[0].len & 0x0f); i++)
                printf("Byte %d of received message :%x!\n", i, cmsg[0].data[i]);
        }
        break;
    } while (1);
    /* ############################################################### */
    retvalue = canIdDelete(rxhandle, 0);
    if (retvalue != NTCAN_SUCCESS)
        printf("canIdDelete() failed with error %d!\n", retvalue);
    printf("function canIdDelete() returned OK !\n");
    /* ############################################################### */
    retvalue = canClose(rxhandle);
    if (retvalue != NTCAN_SUCCESS)
        printf("canClose() failed with error %d!\n", retvalue);
    else
        printf("canClose() returned OK !\n");
    /* ############################################################### */
    return 0;
}