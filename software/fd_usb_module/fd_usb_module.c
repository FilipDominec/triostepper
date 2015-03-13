#include <Python.h>
// TODO this requires "sudo apt-get install python2.7-dev libusb-dev" -> note in readme
// TODO test with python3.2-dev
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "hiddata.h"
#include "usbconfig.h"  /* for device VID, PID, vendor name and product name */

/// Following constants must be common for firmware and software
#define ZERO_POSITION (uint32_t)(1<<30)
// commands TODO SYNC 
#define CMD_MOVE 1
#define CMD_READ_STATUS 2
#define CMD_SET_DRILL_PWM 3
#define CMD_MOTOR_OFF 4
#define CMD_READ_POSITION 5

/* Constant definitions *//*{{{*/
/*}}}*/

static char *usbErrorMessage(int errCode)/*{{{*/
{
static char buffer[80];

    switch(errCode){
        case USBOPEN_ERR_ACCESS:      return "Access to device denied";
        case USBOPEN_ERR_NOTFOUND:    return "The specified device was not found";
        case USBOPEN_ERR_IO:          return "Communication error with device";
        default:
            sprintf(buffer, "Unknown USB error %d", errCode);
            return buffer;
    }
    return NULL;    /* not reached */
}/*}}}*/
static usbDevice_t  *openDevice(void)/*{{{*/
{
usbDevice_t     *dev = NULL;
unsigned char   rawVid[2] = {USB_CFG_VENDOR_ID}, rawPid[2] = {USB_CFG_DEVICE_ID};
char            vendorName[] = {USB_CFG_VENDOR_NAME, 0}, productName[] = {USB_CFG_DEVICE_NAME, 0};
int             vid = rawVid[0] + 256 * rawVid[1];
int             pid = rawPid[0] + 256 * rawPid[1];
int             err;

    if((err = usbhidOpenDevice(&dev, vid, vendorName, pid, productName, 0)) != 0){
        fprintf(stderr, "Error communicating with %s: %s\n", productName, usbErrorMessage(err));
        return NULL;
    }
    return dev;
}/*}}}*/
PyObject* WriteASCII(PyObject* self, PyObject* args)/*{{{*/
{
	usbDevice_t *dev;
	/// Write the command followed by data
	if ((dev = openDevice()) == NULL) {
        	fprintf(stderr, "Error opening device\n");
		return NULL;
	}
	
	/// Process the parameter(s) 
	char        buffer[256];    // 1st byte reserved -> 255 bytes is the maximum command length
    bzero(buffer, sizeof(buffer));
	int len;
    char * s;
	if (!PyArg_ParseTuple(args, "s#", &s, &len)) {
       	fprintf(stderr, "Error: parameter must be one ASCII command\n");
		return NULL;   // FIXME error-prone?
	}	
	

    /// Prepend one character to the buffer
    fprintf(stderr, "S: %s\n", s);
    strcpy(buffer+1, s); buffer[0]='_';
	int err;
        if ((err = usbhidSetReport(dev, (char*)buffer, len+1)) != 0) {
            	fprintf(stderr, "Error sending command: %s\n", usbErrorMessage(err));
		return NULL;
	}
	
    usbhidCloseDevice(dev);
	return Py_BuildValue("s", "OK" );
};/*}}}*/
PyObject* IsBusy(PyObject* self, PyObject* args)/*{{{*/
{
	unsigned char        buffer[3];    
        bzero(buffer, sizeof(buffer));
	
	/// Init the device
	usbDevice_t *dev;
	if ((dev = openDevice()) == NULL) {
        	fprintf(stderr, "Error opening device\n");
		//return NULL;
		return Py_BuildValue("i", 16);
	}
	
	// Wait for the MCU 
	//#ifdef WINDOWS   
		//_sleep(20000);		// in stdlib.h, only the mingw addition
	//#else
		//usleep(20000);		// in unistd.h  (see http://alturl.com/xhm7h)
	//#endif

	/// Read status
	int len = sizeof(buffer);
	int err;
	if ((err = usbhidGetReport(dev, 0, (char*)buffer, &len)) != 0) {		// XXX error prone XXX
		fprintf(stderr, "Error reading data: %s\n", usbErrorMessage(err));
		return NULL;
		//return Py_BuildValue("i", 16);
	}
	//printf("Read data: %d and %d\n", buffer[0], buffer[1]);

	/// Return
	usbhidCloseDevice(dev);
	return Py_BuildValue("i", buffer[1]);

};/*}}}*/

PyObject* Move(PyObject* self, PyObject* args)/*{{{*/
{
	unsigned char        buffer[256];    // 1st byte reserved -> 255 bytes is the maximum command length
        bzero(buffer, sizeof(buffer));
	
	/// Process the parameter(s) 
        unsigned int nx, ny, nz, sx, sy, sz;
	if (!PyArg_ParseTuple(args, "IIIIII", &nx, &ny, &nz, &sx, &sy, &sz)) {
       	fprintf(stderr, "Error: wrong parameters\n");
		return NULL;   // FIXME error-prone?
	}	

	/// Prepare the buffer to be sent
	buffer[0] = 0; 				// this byte is stripped by the USB driver
	buffer[1] = CMD_MOVE;		// ... therefore this byte is received as 1st by the device
	*((uint32_t*) (buffer+ 2)) = (uint32_t) (nx);
	*((uint32_t*) (buffer+ 6)) = (uint32_t) (ny);
	*((uint32_t*) (buffer+10)) = (uint32_t) (nz);
	*((uint32_t*) (buffer+14)) = (uint32_t) (sx);
	*((uint32_t*) (buffer+18)) = (uint32_t) (sy);
	*((uint32_t*) (buffer+22)) = (uint32_t) (sz);

	/// Init the device
	usbDevice_t *dev;
	if ((dev = openDevice()) == NULL) {
        	fprintf(stderr, "Error opening device\n");
		return NULL;
	}

	/// Write the command followed by data
	int err;
	if ((err = usbhidSetReport(dev, (char*)buffer, 1+1+12+12)) != 0) {
		fprintf(stderr, "Error sending command: %s\n", usbErrorMessage(err));
		return NULL;
		}

	/// Read status
	int len = sizeof(buffer);
	if ((err = usbhidGetReport(dev, 0, (char*)buffer, &len)) != 0) {		// XXX error prone XXX
		fprintf(stderr, "Error reading data: %s\n", usbErrorMessage(err));
		return NULL;
		//return Py_BuildValue("i", 16);
	}

	usbhidCloseDevice(dev);
	return Py_BuildValue("i", buffer[1]);
};/*}}}*/
PyObject* SetDrillPwm(PyObject* self, PyObject* args)/*{{{*/
{
	unsigned char        buffer[256];    // 1st byte reserved -> 255 bytes is the maximum command length
    bzero(buffer, sizeof(buffer));
	
	/// Process the parameter(s) 
    uint8_t pwm;
	if (!PyArg_ParseTuple(args, "B", &pwm)) {
       	fprintf(stderr, "Error: wrong parameters\n");
		return NULL;   // FIXME error-prone?
	}	

	/// Prepare the buffer to be sent
	buffer[0] = 0; 						// this byte is stripped by the USB driver
	buffer[1] = CMD_SET_DRILL_PWM;		// ... therefore this byte is received as 1st by the device
	*((uint8_t*) (buffer+ 2)) = (uint8_t) (pwm);

	/// Init the device
	usbDevice_t *dev;
	if ((dev = openDevice()) == NULL) {
        	fprintf(stderr, "Error opening device\n");
		return NULL;
	}

	/// Write the command followed by data
	int err;
        if ((err = usbhidSetReport(dev, (char*)buffer, 1+1+1)) != 0) {
            	fprintf(stderr, "Error sending command: %s\n", usbErrorMessage(err));
		return NULL;
	}
    usbhidCloseDevice(dev);
	return Py_BuildValue("s", "OK" );
};/*}}}*/

// TODO MoveWait, MotorOFF, Recalibrate
PyMethodDef fd_usb_module_methods[] = {/*{{{*/
	{"WriteASCII", (PyCFunction)WriteASCII, METH_VARARGS, "Write an ASCII command to the device"},
	{"IsBusy", (PyCFunction)IsBusy, METH_VARARGS, "Returns 0 if device is idle, nonzero value indicates which axes are to be moved yet"},
	{"Move", (PyCFunction)Move, METH_VARARGS, "Move the tool to specified position (X,Y,Z) in native units, positive integer speed (Vx,Vy,Vz) in native units per PWM cycle"},
	{"SetDrillPwm", (PyCFunction)SetDrillPwm, METH_VARARGS, "Set drill PWM; 0 = off, 16 = full speed"},
	{NULL, NULL}
};/*}}}*/
/* in C++ do not forget extern "C" *//*{{{*/
void initfd_usb_module(void)
{
	Py_InitModule("fd_usb_module", fd_usb_module_methods);
	//, "
}
/*}}}*/

// TODO: alt. way of definition http://docs.python.org/py3k/extending/extending.html#the-module-s-method-table-and-initialization-function 
