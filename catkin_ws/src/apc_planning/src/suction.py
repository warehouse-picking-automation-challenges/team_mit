import u3, u6, ue9
import sys
import time

ErrorMessage = '[Suction] Suction not connected, skipping command: '

def start():
    try:
        d = u3.U3()
        d.writeRegister(5000, 0)
        d.close()
        return True
    except:
        print ErrorMessage, 'start'
        return False

def stop():
    try:
        d = u3.U3()
        d.writeRegister(5000, 5)
        d.close()
        return True
    except:
        print ErrorMessage, 'stop'
        return False
        
def check():
    try:
        d = u3.U3()
        ainValue = d.getAIN(0) 
        d.close()
        #print '[Suction] ainValue:', ainValue
        if ainValue>2.5:
            return True
        else:
            return False
    except:
        print ErrorMessage, 'failed to read suction value'
        return False
        
def loop():
    while True:
        print check()
        time.sleep(.1)

if __name__=='__main__':
    import time
    start()
    time.sleep(2)
    stop()
    time.sleep(2)
    start()
    time.sleep(2)
    stop()
    
    

