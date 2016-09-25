
#ifndef _BLERSSI_H
#define _BLERSSI_H

void blerssi_handle_sysevt(VMINT message, VMINT param);
void send_uart_data(unsigned char data[], unsigned int len);  //  Send data to UART port e.g. "AT$SS=31323334\r".

#endif  //  _BLERSSI_H

