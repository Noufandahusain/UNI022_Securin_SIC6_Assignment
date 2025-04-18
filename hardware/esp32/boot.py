# import machine
# import sys
# import time
# import network
# import urequests
# uart = machine.UART(2, baudrate=115200) #rx2 tx2

# # Global variable to store PPP connection state
# # This will persist across soft resets but not power cycles
# # networkPPP = None
# # ppp = None
# # ppp_connected = False
# DEBUG = True

# def test_http():
#     try:
#         logline("Testing HTTP GET request...")
#         response = urequests.get("https://guthib.com")  # atau URL bebas
#         logline("Status code: {}".format(response.status_code))
#         logline("Response text:")
#         logline(response.text)
#         response.close()
#     except Exception as e:
#         logline("HTTP Request failed: {}".format(e))

# def logline(msg):
#     if DEBUG:
#         print(msg)

# def log(msg):
#     sys.stdout.write(msg)

# def send_atcmd(cmd, uart, wait=500, tries=10):
#     uart.write(cmd + '\r\n')
#     time.sleep_ms(wait)
    
#     for _ in range(tries):
#         resp = uart.read()
#         if resp:
#             logline(resp.decode('utf-8') if isinstance(resp, bytes) else str(resp))
#             return resp
#         else:
#             log('.')
#             time.sleep_ms(wait)
#     else:
#         raise Exception("No response from modem: " + cmd)

# def ppp_connect():
#     uart = machine.UART(2, baudrate=115200)
#     # global ppp_connected
#     # global ppp
#     # If already connected, return the existing connection
#     try:
#         ppp = network.PPP(uart)
#         if ppp.isconnected():
#             logline("PPP already connected!")
#             logline("IP: {}".format(ppp.ifconfig()))
#             ppp_connected = True
#             return ppp
#     except:
#         logline("Checking connection status failed")
    
#     time.sleep(1)

#     logline("Sending initial AT commands...")
#     send_atcmd("AT", uart)
#     send_atcmd("ATE0", uart)
#     send_atcmd("AT+CFUN=1", uart)
#     send_atcmd("AT+CPIN?", uart)
#     send_atcmd("AT+CREG?", uart)
#     send_atcmd("AT+CGATT=1", uart)

#     apn = "internet"
#     send_atcmd('AT+CGDCONT=1,"IP","{}"'.format(apn), uart)
#     time.sleep(1)

#     logline("Starting PPP connection...")
#     resp = send_atcmd('ATD*99#', uart, wait=1000)

#     ppp = network.PPP(uart)
#     ppp.active(True)
#     ppp.connect()

#     for _ in range(30):
#         if ppp.isconnected():
#             logline("PPP connected!")
#             logline("IP: {}".format(ppp.ifconfig()))
#             ppp_connected = True
#             return ppp
#         else:
#             log('wait bg\n')
#             time.sleep(1)
#     else:
#         ppp.close()
#         logline("PPP connection timed out!")
#         raise Exception("PPP connection failed!")


# try:
#     networkPPP = ppp_connect()
#     test_http()
# except Exception as e:
#     logline("ERROR: {}".format(e))


import time

import network

import machine as m

sta_if = network.WLAN(network.STA_IF); sta_if.active(True)

sta_if.scan() 

sta_if.connect("GL88_ATAS", "miliknegara")

sta_if.isconnected()

time.sleep(3)