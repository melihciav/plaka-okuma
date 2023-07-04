from pirc522 import RFID  #Rfıd kütüphanesi scripte dahil edilmiştir.
from PIL import Image, ImageOps  #Image kütüphanesi scripte dahil edilmiştir.
import cv2  #cv2 kütüphanesi scripte dahil edilmiştir.
import numpy as np  #numpy kütüphanesi scripte dahil edilmiştir.
import pytesseract  #pytesseract kütüphanesi scripte dahil edilmiştir.
import imutils  #imutils kütüphanesi scripte dahil edilmiştir.
import signal  #signal kütüphanesi scripte dahil edilmiştir.
import time  #time kütüphanesi scripte dahil edilmiştir.
import RPi.GPIO as GPIO  #GPIO kütüphanesi scripte dahil edilmiştir.

#Pin numaralarının tanımlanması
TRIG = 36
ECHO = 38
yled=7
kled=13
sled=11
servoPIN = 35
LCD_RS = 37
LCD_E = 40
LCD_D4 = 33 
LCD_D5 = 32
LCD_D6 = 31
LCD_D7 = 29
LED_ON = 15
LCD_WIDTH = 16  #LCD’nin bir satırdaki maksimum karakter sayısını belirtir
LCD_CHR = True
LCD_CMD = False
LCD_LINE_1 = 0x80  #LCD ilk satırının adresi
LCD_LINE_2 = 0xC0  #LCD ikinci satırının adresi 
 

#Zaman sabitleri
E_PULSE = 0.00005
E_DELAY = 0.00005

GPIO.setwarnings(False)  #Küçük hataları yok sayar
GPIO.setmode(GPIO.BOARD)  #Raspberry’i board pin dizilimine göre ayarlar
GPIO.setup(yled,GPIO.OUT)  #Yeşil led çıkışı ayarlanır
GPIO.setup(sled,GPIO.OUT)  #Sarı led çıkışı ayarlanır
GPIO.setup(kled,GPIO.OUT)  #Kırmızı led çıkışı ayarlanır
GPIO.setup(TRIG,GPIO.OUT)  #Ultrasonik sensörün ses dalgalarını gönderen Trig pin çıkışı
GPIO.setup(ECHO,GPIO.IN)  #Ultrasonik sensörün ses dalgalarını alan Echo pin çıkışı
GPIO.setup(servoPIN, GPIO.OUT)  #Servo motorun pin çıkışı

def main():
 #LCD hazirkik fonksiyonunun cagrilmasi
    lcd_init()
 # Arka plan aydınlatması test
    GPIO.output(LED_ON, True)
    time.sleep(1)
    GPIO.output(LED_ON, False)
    time.sleep(1)
    GPIO.output(LED_ON, True)
    time.sleep(1)
def lcd_init():
    GPIO.setwarnings(False) 
    GPIO.setup(LCD_E, GPIO.OUT) 
    GPIO.setup(LCD_RS, GPIO.OUT) 
    GPIO.setup(LCD_D4, GPIO.OUT) 
    GPIO.setup(LCD_D5, GPIO.OUT) 
    GPIO.setup(LCD_D6, GPIO.OUT) 
    GPIO.setup(LCD_D7, GPIO.OUT) 
    GPIO.setup(LED_ON, GPIO.OUT)  #Arka plan aydınlatması
  
#LCD Pinlerine veri gonderimi
    lcd_byte(0x33,LCD_CMD)
    lcd_byte(0x32,LCD_CMD)
    lcd_byte(0x28,LCD_CMD)
    lcd_byte(0x0C,LCD_CMD) 
    lcd_byte(0x06,LCD_CMD)
    lcd_byte(0x01,LCD_CMD) 
 
def lcd_string(message,style):
    if style==1:
        message = message.ljust(LCD_WIDTH," ") 
    elif style==2:
        message = message.center(LCD_WIDTH," ")
    elif style==3:
        message = message.rjust(LCD_WIDTH," ")
    for i in range(LCD_WIDTH):
        lcd_byte(ord(message[i]),LCD_CHR)
 
def lcd_byte(bits, mode): 
    GPIO.output(LCD_RS, mode) 
    GPIO.output(LCD_D4, False)
    GPIO.output(LCD_D5, False)
    GPIO.output(LCD_D6, False)
    GPIO.output(LCD_D7, False)
    if bits&0x10==0x10:
        GPIO.output(LCD_D4, True)
    if bits&0x20==0x20:
        GPIO.output(LCD_D5, True)
    if bits&0x40==0x40:
        GPIO.output(LCD_D6, True)
    if bits&0x80==0x80:
        GPIO.output(LCD_D7, True)
    time.sleep(E_DELAY) 
    GPIO.output(LCD_E, True) 
    time.sleep(E_PULSE)
    GPIO.output(LCD_E, False) 
    time.sleep(E_DELAY) 
    GPIO.output(LCD_D4, False)
    GPIO.output(LCD_D5, False)
    GPIO.output(LCD_D6, False)
    GPIO.output(LCD_D7, False)
    if bits&0x01==0x01:
        GPIO.output(LCD_D4, True)
    if bits&0x02==0x02:
        GPIO.output(LCD_D5, True)
    if bits&0x04==0x04:
        GPIO.output(LCD_D6, True)
    if bits&0x08==0x08:
        GPIO.output(LCD_D7, True)
    time.sleep(E_DELAY) 
    GPIO.output(LCD_E, True) 
    time.sleep(E_PULSE)
    GPIO.output(LCD_E, False) 
    time.sleep(E_DELAY) 
if __name__ == '__main__':
    main()
oku=RFID()
util=oku.util()
util.debug=True
GPIO.output(yled,False)
GPIO.output(kled,False)
GPIO.setup(10, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
p = GPIO.PWM(servoPIN, 50)  #Servo pinin bağlı olduğu çıkış 50Hz bir frekans ile tetiklemiştir.

print ("Program Açılıyor..")

#LCD ortalanmış yazı
lcd_byte(LCD_LINE_1, LCD_CMD)
lcd_string("Program",2)
lcd_byte(LCD_LINE_2, LCD_CMD)
lcd_string("Aciliyor",2)
time.sleep(2)

while True:
    GPIO.output(TRIG, False)
    #Buton kodları
    A=0  
    if GPIO.input(10) == GPIO.HIGH:
        A=1
        print("Kartınızı okutun..")
        lcd_byte(LCD_LINE_1, LCD_CMD)
        lcd_string("Kartinizi",2)
        lcd_byte(LCD_LINE_2, LCD_CMD)
        lcd_string("Okutun",2)
        time.sleep(0.5)
    time.sleep(1)
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)
    while GPIO.input(ECHO)==0:
        pulse_start = time.time()
    while GPIO.input(ECHO)==1:
        pulse_end = time.time()
        pulse_duration = pulse_end - pulse_start
        distance = pulse_duration*17150
        distance = round(distance, 2)
    if distance<5:
        print("Araç Geldi, Kamera Açılıyor..")
        lcd_byte(LCD_LINE_1, LCD_CMD)
        lcd_string("Arac Geldi",2)
        lcd_byte(LCD_LINE_2, LCD_CMD)
        lcd_string("Kamera Aciliyor",2)
        time.sleep(0.5)
        cap = cv2.VideoCapture(0)
        i=0
        while(cap.isOpened()):
            ret, frame = cap.read()
            if ret == False:
                break
            cv2.imwrite('kare'+str(i)+'.jpg',frame)
            i+=1
            if i == 2:
                break
        cap.release()
        img = cv2.imread('kare0.jpg',cv2.IMREAD_COLOR)
        img = cv2.resize(img, (600,400))
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) 
        gray = cv2.bilateralFilter(gray, 13, 15, 0) 

        edged = cv2.Canny(gray, 30, 200) 
        contours = cv2.findContours(edged.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours = imutils.grab_contours(contours)
        contours = sorted(contours, key = cv2.contourArea, reverse = True)[:10]
        screenCnt = None
        for c in contours:
            
            peri = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, 0.018 * peri, True)
         
            if len(approx) == 4:
                screenCnt = approx
                break

        if screenCnt is None:
            detected = 0
            print ("No contour detected")
        else:
             detected = 1

        if detected == 1:
            cv2.drawContours(img, [screenCnt], -1, (0, 0, 255), 3)

        mask = np.zeros(gray.shape,np.uint8)
        new_image = cv2.drawContours(mask,[screenCnt],0,255,-1,)
        new_image = cv2.bitwise_and(img,img,mask=mask)

        (x, y) = np.where(mask == 255)
        (topx, topy) = (np.min(x), np.min(y))
        (bottomx, bottomy) = (np.max(x), np.max(y))
        Cropped = gray[topx:bottomx+1, topy:bottomy+1]

        text = pytesseract.image_to_string(Cropped, config='--psm 11')
        print("Plaka Tespit Edildi:",text)
        str(text)
        p1=text.strip()
        p2=p1.upper()
        p3="".join(filter(str.isalnum, p2))

        Cropped = cv2.resize(Cropped,(400,200))
        cv2.imwrite('kare1.jpg',Cropped)
        img = Image.open(r"/home/pi/Desktop/kare1.jpg").convert("L")
        img1 = ImageOps.colorize(img, black ="white", white ="black")
        img1.show()
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        konum = ("/home/pi/Desktop/plakabilgileri.txt")
        dosya = open(konum, "r") 
        aranan = p3.upper()
        print("Son hali",p3)
        aranan_varmi = dosya.read().find(aranan)
        if aranan_varmi != -1:
            print("Plaka tanındı, kapı açılıyor.")
            lcd_byte(LCD_LINE_1, LCD_CMD)
            lcd_string("Plaka Tanindi",2)
            lcd_byte(LCD_LINE_2, LCD_CMD)
            lcd_string("Kapi Aciliyor",2)
            time.sleep(2)
            p.start(10)  #Servo sinyali 10ms ile başlatılmıştır.
            try:
                while True:
                    p.ChangeDutyCycle(5)
                    time.sleep(3)
                    p.ChangeDutyCycle(10)
                    break

            except KeyboardInterrupt:
                p.stop()
                GPIO.cleanup()
        else: 
            print("XXX Plaka tanınmadı XXX")
            lcd_byte(LCD_LINE_1, LCD_CMD)
            lcd_string("Plaka",2)
            lcd_byte(LCD_LINE_2, LCD_CMD)
            lcd_string("Taninmadi",2)
            time.sleep(2)
        dosya.close() 

    elif A==1:
        while True:
            try:
                GPIO.output(sled,True)
                oku.wait_for_tag()
                (error,data)=oku.request()
                if not error:
                    print("\nOkuma başarılı")
                    (error,kartid)=oku.anticoll()
                    if not error:
                        kart_uid=str(kartid[0])+" "+str(kartid[1])+" "+str(kartid[2])+" "+str(kartid[3])+" "+str(kartid[4])
                        print(kart_uid)
                        kled=13
                        sled=11
                        if kart_uid=="51 56 145 6 156":
                            print("Doğru kart, kayıt sistemi açılıyor.")
                            lcd_byte(LCD_LINE_1, LCD_CMD)
                            lcd_string("Dogru Kart Kayit",2)
                            lcd_byte(LCD_LINE_2, LCD_CMD)
                            lcd_string("Sistemi Aciliyor",2)

                            GPIO.output(yled,True)
                            GPIO.output(kled,False)
                            GPIO.output(sled,False)
                            time.sleep(2)
                            GPIO.output(yled,False)
                            GPIO.output(sled,True)
                            lcd_byte(LCD_LINE_1, LCD_CMD)
                            lcd_string("Kayit",2)
                            lcd_byte(LCD_LINE_2, LCD_CMD)
                            lcd_string("Sistemi",2)
                            def bilgileri_gir():
                                ad = input("Araç Sahibi:")
                                plaka = str(input("Aracın Plakası:"))
                                with open("plakabilgileri.txt","a", encoding="utf-8") as file:
                                    file.write(ad+" :"+plaka+"\n")
                            def plakasorgula():
                                konum = ("/home/pi/Desktop/plakabilgileri.txt")
                                dosya = open(konum, "r") 
                                aranan = input("Sorgulanacak plaka numarası:").upper()
                                aranan_varmi = dosya.read().find(aranan)
                                if aranan_varmi != -1:
                                    print("Plaka tanındı, kapı açılıyor.")
                                    lcd_byte(LCD_LINE_1, LCD_CMD)
                                    lcd_string("Plaka Tanindi",2)
                                    lcd_byte(LCD_LINE_2, LCD_CMD)
                                    lcd_string("Kapi Aciliyor",2)
                                    GPIO.setwarnings(False)
                                    p.start(10)
                                    try:
                                        while True:
                                            p.ChangeDutyCycle(5)
                                            time.sleep(3)
                                            p.ChangeDutyCycle(10)
                                            break
                                                
                                    except KeyboardInterrupt:
                                        p.stop()
                                        GPIO.cleanup()
                                elif aranan_varmi == -1:
                                    print("Plaka tanınmadı.")
                                    lcd_byte(LCD_LINE_1, LCD_CMD)
                                    lcd_string("Plaka",2)
                                    lcd_byte(LCD_LINE_2, LCD_CMD)
                                    lcd_string("Taninmadi",2)
                                    GPIO.setwarnings(False)
                                    p.start(10)
                                    try:
                                        while True:
                                            p.ChangeDutyCycle(10)
                                            time.sleep(1)
                                            p.ChangeDutyCycle(10)
                                            break
                                                
                                    except KeyboardInterrupt:
                                        p.stop()
                                        GPIO.cleanup()
                                    
                                    

                            while True:
                                islem = input(" \nKAYIT SİSTEMİ\n1-Yeni Kayıt:\n2-Plaka Sorgula:\n3-Uygulamadan Çıkış.\n")
                                if islem == "1":
                                    bilgileri_gir()
                                elif islem == "2":
                                    plakasorgula()
                                elif islem == "3":
                                    break
                            break
                        else:
                            print("XXX Yanlış Kart XXX")
                            lcd_byte(LCD_LINE_1, LCD_CMD)
                            lcd_string("Yanlis",2)
                            lcd_byte(LCD_LINE_2, LCD_CMD)
                            lcd_string("Kart",2)
                            GPIO.output(yled,False)
                            GPIO.output(kled, True)
                            time.sleep(3)
                            GPIO.output(kled, False)
                            break
            except KeyboardInterrupt:
                GPIO.cleanup()
                break
    else:
        lcd_byte(LCD_LINE_1, LCD_CMD)
        lcd_string("Araba yok",2)
        lcd_byte(LCD_LINE_2, LCD_CMD)
        lcd_string("",2)
        time.sleep(0.5)
        print("Araba yok.")
