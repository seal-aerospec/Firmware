# MVP Beta
## V2.3

Turn off Cellular

## V2.2.2

### Bug fixing, Legacy libraries removed, officially ported for Beta Devices

Removed GPS and RTC libraries and all of the related functions that were causing datestamp errors on beta devices. As of now the 

## V2.2.1

### BLE Flags

The Device will now send flags to the phone app for the historical data management.

## V2.2.0

### BLE Backup & Readout

The device can now back up samples to the SD card if it hasn't been connected. After detecting the device does not have a bluetooth connection, a second text file will be generated on the SD card containing averaged backup data. The data is an aggregate backup of samples over a 15 minute period. After the device re-establishes a BLE connection, the device will send all of the backup samples to the phone. This could take on average 30 seconds to a few minutes - it is also a blocking call meaning the device will do nothing else during this time. 

Tested on Robert's phone at this time. 


## V2.1.0

### Sampling Overhauled

Sampling has been overhauled for this new release of the MVP Beta. Essentially, samples will now be globally accessible via a struct. Sampling, writing to the SD card and publishing functionalities have been made independent, meaning you no longer need one to do the other. This should make it easier for BLE sampling/writing. 

`take_sample()` will sample all of the peripheral devices and place the latest sample in a global struct. This is how samples will be written and displayed. New samples can only be taken when this method is called. 


### Averaging

Averaging has also been changed. Now instead of holding a running average,  normalizing the running average and sending that specific struct of data, I've pulled out a defined average sample called `average_sample`. This struct will hold the last known average sampled data. So now a running average can continue without losing the last `average_sample`. This will make it easier to publish both to the cloud and over BLE since technically that set of average data should be exactly the same (if ble wants average data). This also simplified some of the code. 


### BLE

BLE functionality is included now, mainly it is boilerplate code from Charlie. It has yet to be tested, Charlie should overlook the code before giving it the final ok. 

### Other Changes

- GPS can now be enabled/disabled with a global flag. For now, it is disabled by default. 
- Error checking and mitigation has been removed. So far, those issues have been resolved by changes. 
- Time Sync has been removed for now, can be implemented given the overhaul. 
- Duty cycle functionality has been revamped as well to match the BAC version. Settings 1-3 can set the duty cycles. 
- Data being displayed to the screen is the last sample taken, instead of taking its own sample. 
- The number of samples N per averaging is no longer accessible to the cloud. Since duty cycles have been streamlined, you can generally figure out how many samples are in each average. Can also record the amount of samples in an average if necessary, since they are now a part of each sample struct. 
- Formatting for each sample has been updated. Most values only go to 2-3 decimal places. GPS resolution has been fixed to 4 decimals but this can be changed later. 



