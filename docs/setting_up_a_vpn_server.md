# Setting Up an Openvpn Server to Create a Virtual Robotics Lab Network 

***WORK IN PROGRESS***

Since the beginning of the pandemic, roboticists have been stuggling to do use their lab resources and do networking in a secure way. By using a VPN we can setup a network which mimics having all your devices on the same network in your lab. A VPN will encrypt all traffic allowing you to securely control simulation computers or robots remotely as long as you have an internet connection.

![vpn co-ops](imgs/vpn_con_ops.png)

## VPN Server

The first thing we need to setup is a VPN server. This is a computer that will manage who can connect and will route all VPN traffic. In this tutorial I will use digital ocean to setup the vpn server. For $5 a month we can create a linux node that we can use as our VPN server. 

### Digital Ocean Discounts

If you are a currently a student you can redeem $50 worth of free credits by using the github student developer pack. This is enough to host a VPN server for 10 months! [Github Student Developer Pack](https://education.github.com/pack)

If you are not a student, you can use my affiliate link which will grant you $100 of credits for 60 days. This isn't as good as the student discount, but at least we can host a VPN for 2 months free. [Digital Ocean $100 for 60 days credit](https://m.do.co/c/6752af521fd4)

### Creating a Digital Ocean Node 

In the top left corner click `new project`. Fill out relvent information. Then click `Get Started with a Droplet`. 

A basic ubuntu 20.04 node will be perfect for this application.
![node settings](imgs/node_settings1.JPG)

- select the data center closest to your location 
- add your ssh key by following the instructions under `New SSH Key`

We should be ready to create our droplet!

### Installing Openvpn 

We will be using the easy install script.

follow the bellow tutorial 
https://nextsouls.com/how-to-install-openvpn-using-script-installer/

## Client Setup 

On the VPN Server we can re-runt the install script to generate client connect files.

## Installing Openvpn Clients 

Download client: https://openvpn.net/download-open-vpn/

import openvpn profile

![ovpn client](imgs/ovpn_client.JPG)