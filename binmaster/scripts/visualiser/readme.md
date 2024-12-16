## Binmaster Testing Tools
### System Details
Login: ubuntu  
Password: intelnuc  
Address (Bonjour/Avahi/Zeroconf): dipbob.local

### WiFi
System is currently setup to join a WiFi network with the following details:  
APN: AP1  
Password: Bazinga1

### SSH
#### Windows
Use [putty](https://www.chiark.greenend.org.uk/~sgtatham/putty/latest.html) with above details. 
Note that bonjour may need to be install to resolve zerconf address. See [Adafruit](https://learn.adafruit.com/bonjour-zeroconf-networking-for-windows-and-linux)
for instructions. Note getting the latest requires downloading itunes and using 7-Zip to extract the bonjour.msi installer. 
#### Linux

```bash
ssh ubuntu@dipbob.local
```

### Requirements
Note all requirements will be installed in the next step. Just listed for reference. 
#### System
* Python 3
* R

#### Python
* PySerial
* Dash
* Scipy
* Numpy
* Plotly
* Flask

#### R
* changepoint (NOTE GPL LICENCE)

### Setup Visualiser
* Setup SD card with Ubuntu 20.04lts 64bit image using [Raspberry Pi Imager](https://www.raspberrypi.org/downloads/)  
* Setup WiFi access as per [Ubuntu Instructions](https://ubuntu.com/tutorials/how-to-install-ubuntu-on-your-raspberry-pi#3-wifi-or-ethernet)
* Install SD card
* Connect monitor (via micro HDMI) and keyboard (via USB)
* Connect USB C power supply (>=2A)
* Default login is username: ubuntu, password: ubuntu. May ask you to select new password
* Run following:
```bash
sudo apt update
sudo apt upgrade
sudo apt install r-base python3-pip python3-numpy python3-scipy python3-rpy2
sudo pip3 install pyserial plotly dash
sudo -i R
install.package("changepoint")
q()
n
cd  ~/
git clone https://[USERNAME]@bitbucket.org/orica/surface-automation.git
cd surface-automation/binmaster/scripts/visualiser/
python3 setup_db.py
```

### Running Visualiser
* Run WiFi hotspot on phone as detailed above
* Insert Makita Battery
* Wait a moment or two for the Pi to boot
* SSH in as above
* Run following over SSH
```bash
cd surface-automation/binmaster/scripts/visualiser/
python3 visualiser.py
```
* Navigate to dipbob.local:8050 on browser
* Command as desired
* Once testing complete ctrl-c to kill visualiser.py and run following over SSH
```bash
sudo shutdown now
```
* Wait moment or two for the Pi to shutdown
* Disconnect Makita Battery

### Retrieving Logs
#### Windows
Any STFP explorer such as Filezilla will be able to connect via the same details as above to access
the filesystem and download logs.

Note the logs are available at 
```bash
/home/ubuntu/surface-automation/binmaster/scripts/visualiser/logs
```  

And the SQlite database at
```bash
/home/ubuntu/surface-automation/binmaster/scripts/assets/logging.db
```

#### Linux

```bash
scp ubuntu@dipbob.local:/home/ubuntu/surface-automation/binmaster/scripts/visualiser/assets/logging.db .
scp ubuntu@dipbob.local:/home/ubuntu/surface-automation/binmaster/scripts/visualiser/logs/* .
```

### Offline Analysis
#### Analyse.py
Analyse.py will run the depth and water detection analysis on a single log file. A graph will be displayed as
the output.  

The script would typically be invoked in a manner similar to below.  

```bash
python3 anaylse.py -f logs/[uid].json
```

#### Analyse_all.py
Analyse_all.py will run depth and water detection analysis on all cycles present in the database. A csv file will be
produced detailing the results. Unlike analyse.py no graph outputs are produced. 

The script would typically be invoked in a manner similar to below. Note the -p argument can be used to specify
the number of parallel process that will be spawned to accelerate analysis (default of 8). 

```bash
python3 analyse_all.py
```

#### Analyser.R
Analyser.R is similar to analyse.p, but is a leftover artifact from earlier testing of the various changepoint detection
methods. Note water detection, while in python, calls the R changepoint library. As such this script is off little use. 

The script would typically invoked in a manner similar to below.

```bash
Rscript analyser.R
```

### Util Code
#### Cycle.py
Will simply request the binmaster on the provided serial port to perform a cycle and capture the resulting
encoder information into a json log file. Primarily called by visualiser.py as a workaround to ensure that 
the polling of the serial port to capture the stream of data of the UART is not interrupted resulting in dropped
information (hack job). Though no reason it can't be used as follows

```bash
python3 cycle.py -f [filename].json
``` 

#### Setup_db.py
When a fresh install of software is done, this will create a new SQLite database with the correct table structure
for use by visualiser.py. Typically only ever invoked as part of the setup instructions above. 