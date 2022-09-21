import serial
import time
import find_arduino as fa

def test_arduino():
    ard = fa.Connect()
    counter = -255
    while counter <= 256:
        leftmotor = str(counter)
        rightmotor = str(-counter)
        cmd = '%' + leftmotor + '&' + rightmotor + '*'
        print(str.encode(cmd))
        ard.write(str.encode(cmd))
        counter += 1
        time.sleep(0.02)
        # print(ard.readline())
        # print(ard.readline())
        # print(ard.readline())

if __name__ == '__main__':
    test_arduino()