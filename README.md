# Term-Team6
## 스마트 시리얼 디스펜서  
###### 정민규, 조동인
***
- __개요__  
각 모듈들을 이용하여 자동으로 시리얼을 만들어주는 장치

- __시나리오__  
특정시간이 되면 자동으로 시리얼과 우유를 배출하고 스피커로 사용자에게 알려준다.  
사용자가 터치센서(시리얼)를 누름 -> 그릇유무 확인 -> 시리얼 배출  
사용자가 터치센서(우유)를 누름 -> 그릇유무 확인 -> 일정 수위가 될 때까지 우유 

- __기능__  
  특정 시간에 자동으로 시리얼과 우유 배출 후 스피커로 알려줌.  
  수동으로 시리얼과 우유 조절 기능.   
  시리얼 디스펜서 바닥에 그릇이 없으면 시리얼과 우유 배출 안함.   
  그릇에 우유가 일정 수위 이상 차면 시리얼과 우유 배출 안함  

- __사용 센서 및 모듈__  
  서보 모터(시리얼 배출), 모터드라이버, 워터 펌프 모터(우유 배출),  
  초음파 센서(그릇 유무 확인), 수위 측정 센서(일정 수위조건 만족 확인),  
  터치 센서(추가 시리얼, 우유 배출), 스피커(준비가 완료 되었음을 알림)
  
- __동작 흐름도__  
![image](https://user-images.githubusercontent.com/31990118/120493927-5461d180-c3f6-11eb-9f23-5b95f4cfc136.png)
