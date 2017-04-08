// Interpreter.h
// Interpreter for UART
// TA: Daniel Leach

/*---------------------ProcessCommand---------------------
* Function: ProcessCommand is a busy wait function that waits on the UART_InString to be available. 
*	Once ready, we process the string from UART. 
*/
void ProcessCommand(void); 

/*---------------------OutCRLF---------------------
* Function: New Line
* UART output: CRLF
*/
void OutCRLF(void);
