# HW3

LED1 代表 gesture_UI mode;
LED2 代表 tilt_angle mode;
LED3 代表 偵測reference angle;
將mbed板與uLCD接上電腦

1. 開 new terminal 

2. mkdir -p ~/ee2405new

3. cp -r ~/ee2405/mbed-os ~/ee2405new

4. cd ~/ee2405new

5. mbed compile --library --no-archive -t GCC_ARM -m B_L4S5I_IOT01A --build ~/ee2405new/mbed-os-build2

6. 開 new terminal (此terminal以terminal1代稱)

7. cd ~/HW3/src/model_depoly

8. sudo mbed compile --source . --source ~/ee2405new/mbed-os-build2/ -m B_L4S5I_IOT01A -t GCC_ARM --profile tflite.json -ff 

9.  teriminal1 會顯示wifi連線，與ip位址。

10. 開 new terminal (此terminal以terminal2代稱)

11. cd ~/HW3/src/model_depoly

12. sudo python3 mqtt_client.py

13. terminal2 會顯示會顯示wifi連線，與ip位址，再來傳五次訊息至terminal1，terminal1會顯示接收到

14. 在terminal1 輸入 /gesture_UI/run，按下enter，此時LED1會亮起，螢幕上會有 "Set up successful...\n"，這時候揮動mbed板，就會增加設定的角度，角度會顯示在uLCD上
 
15. 選擇完角度之後，按下USER_BUTTON，uLCD會顯示CONFIRM，terminal2會接收到設定的角度，LED1會熄滅
 
16. 在terminal1 輸入 /tilt_angle/run，按下enter，此時LED3會亮起，請在兩秒內將mbed板子平放以利偵測基準角度，待LED3熄滅，LED2會亮起，然後會在terminal1上顯示當前角度
 
17. 緩緩傾斜mbed板，當角度超過設定角度時，會將超過的角度數值傳到terminal2
 
18. 當連續超過10次偵測到角度超過設定角度時，LED2會熄滅，偵測角度會終止，此時可以在RPC繼續輸入東西
