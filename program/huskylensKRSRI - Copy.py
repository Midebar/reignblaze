import time
from huskylib import HuskyLensLibrary

# ============================ START SETUP ============================
hl = HuskyLensLibrary("I2C","", address=0x32)
# ============================= END SETUP =============================



# ============================ START LOOP 0 ============================
loop0 = 1
while(loop0 == 1):

    # Switch Algorithm to Object Classification
    hl.algorthim("ALGORITHM_OBJECT_CLASSIFICATION")        
    
    # Load Learned Object Classification Data From Memory
    hl.loadModelFromSDCard(1)                                       # ..Model..(1) = Obj. Class  |  ..Model..(2) = Obj. Track. Korban  |  ..Model..(3) = Obj. Track. Lilin.
    
    #Apakah Ada Learned Object == Lilin
    if (hl.blocks().ID == 2):                                       # ID(0) = Objek Belum Terpelajari  |  ID(1) = Korban  |  ID(2) = Lilin.

        # Switch Algorithm to Object Tracking
        hl.algorthim("ALGORITHM_OBJECT_TRACKING")
        
        # Load Learned Object Tracking Lilin Data From Memory
        hl.loadModelFromSDCard(3)

        # ===================== START LOOP 02 =====================
        loop02 = 1
        while (loop02 == 1):
        
            # Apakah 110 <= blocks().width <= 130?
            if (110 <= hl.blocks().width and hl.blocks().width <= 130):

                # Robot Melakukan Pemadaman
                numEnter = 'h'
                print(fungsi(numEnter)))
                time.sleep(2)

                # Out From LOOP 02
                loop02 = 0

            # Else, Apakah blocks().width < 110?
            elif (hl.blocks().width < 110):

                # Robot Bergerak Ke Depan 1 Langkah
                numEnter = 'a'
                print(fungsi(numEnter)))
                time.sleep(2)

            # Else
            else:

                #Robot Bergerak Ke Belakang 1 Langkah
                numEnter = 'b'
                print(fungsi(numEnter)))
                time.sleep(2)
        # ====================== END LOOP 02 ======================
    # Apakah Ada Learned Object == Korban
    elif (hl.blocks().ID == 1):
        
        # Switch Algorithm to Object Tracking
        hl.algorthim("ALGORITHM_OBJECT_TRACKING")
        
        # Load Learned Object Tracking Korban Data From Memory
        hl.loadModelFromSDCard(2)
        
        # ===================== START LOOP 01 =====================
        loop01 = 1
        while(loop01 == 1):
        
            # Apakah 140 <= Blocks().x <= 160?
            if (140 <= hl.blocks().x and hl.blocks().x <= 160):
            
                # Apakah 110 <= Blocks().width <= 130?
                if (110 <= hl.blocks().width and hl.blocks().width <= 130):
                
                    # Robot Bergerak Turun, Menjalankan Fungsi Mengambil Korban, Kembali Naik, Evakuasi
                    numEnter = 'i'
                    print(fungsi(numEnter)))
                    time.sleep(2)

                    # Out From LOOP 01
                    loop01 = 0

                # Else, Apakah blocks().width < 110?
                elif (hl.blocks().width < 110):
                    
                    # Robot Bergerak Ke Depan 1 Langkah
                    numEnter = 'a'
                    print(fungsi(numEnter)))
                    time.sleep(2)

                # Else
                else:

                    # Robot Bergerak Ke Belakang 1 Langkah
                    numEnter = 'b'
                    print(fungsi(numEnter)))
                    time.sleep(2)
        
            # Else, Apakah X.Objek < 140?
            elif (hl.blocks().x < 140):

                # Robot Bergerak Ke Kanan 1 Langkah
                numEnter = 'e'
                print(fungsi(numEnter)))
                time.sleep(2)

            # Else
            else:

                # Robot Bergerak Ke Kiri 1 Langkah
                numEnter = 'f'
                print(fungsi(numEnter)))
                time.sleep(2)
        # ====================== END LOOP 01 ======================
# ============================= END LOOP 0 =============================