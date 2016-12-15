# CycleOpsPro300PTtoANTPlus
Read in the raw data from the CycleOps Pro 300PT Indoor Cycle wheel and output it as a ANT+ signal using an Arduino and Nordic nRF24AP

## Background
I got a really good deal on Craigslist on this indoor bike. I had also recently become addicted to Zwift and really wanted to be able to use this with Zwift rather than use my bike on a trainer. The included computer refused to save data past about 10 minutes making it all but worthless for saving training sessions. As I find time I'll be documenting the reverse engineering and build at [https://blinkyme.wordpress.com/2016/12/13/cycleops-pro300-pt-to-antplus/](https://blinkyme.wordpress.com/2016/12/13/cycleops-pro300-pt-to-antplus/)

## Step 1 Try to ask CycleOps
I tried to email CycleOps tech support to see if I could take the easy route and pay to upgrade this to an ANT+ compatible power meter. Turned out the version I had was too old and couldn’t be upgraded. The power meter is welded to the fly wheel. Newer version have the power meter bolted to a center hub that can be removed. I directly asked for information on the protocol it uses to communicate with the head unit and was quickly told that they will not be sharing the proprietary protocol with me. 

## Step 2, Hook up the oscilloscope
