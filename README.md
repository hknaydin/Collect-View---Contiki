Hello in this article, I will check the battery on your cooja with collect-view. When I received a lot of questions on the Internet, I thought to explain it in detail. Throughout my master’s thesis, this structure worked very well. I’m sure you’il be able to use it. I have summarized this topic with a short video on my Youtube page.

https://youtu.be/P_MakgoFgYk

The codes were implemented with the C language on the Contiki operating system.

The collect-view folder is available under Contiki OS. With this folder we can do all our operations easily. Under the folder there are samples made for sky and z1 motels. Since I’m working on exp5438, I’ve made some adjustments to the Makefile (Makefile.collect-view). We need to add collect-view-exp5438.c and collect-view-exp5438.h.

    collect-view_src = collect-view.c

    ifeq ($(TARGET), exp5438)
    collect-view_src += collect-view-exp5438.c
    else
    ifeq ($(TARGET), sky)
    collect-view_src += collect-view-sky.c
    else
    ifeq ($(TARGET), z1)
    collect-view_src += collect-view-z1.c
    endif
    endif
    endif

https://yazilimdnyasi.files.wordpress.com/2019/10/oie_kgyc3synoz5k.png?w=788

I used the files of sky and z1 motels for exp5438 device. You can observe the results by making the necessary updates for the device you have used. When the files are examined, the energy values used are sent to a central server and the results are plotted with java file on the Cooja emulator.

The Makefile file in udp-client is updated as follows.

    APPS + = powertrace collect-view

With the help of udp-sink and udp-sender files available in Contiki, server and client files are replaced with the appropriate codes. Important functions are underlined. Edit udp-client file according to udp-server. Since there are some additions in my code, everyone should edit the codes by themselves.
