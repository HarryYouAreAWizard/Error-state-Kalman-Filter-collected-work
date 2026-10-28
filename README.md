

to compile and test Kalman filter:

sh compile_and_run.sh


look at sim_and_livetest/figures/ESKF_true_vs_estimated.png to see results


To run live test:

download the phyphox app to your phone

start a wifi-hotspot from your phone, and log onto that hotspot from your pc

set up an experiment with accelerometer, gyroscope, magentometer and gnss (location)

activate remote access

find the IP in the bottom of the phyphox screen

write IP into "live_phone_data_test.py" in the top of the script

run the live_phone_data_test.py script. 


Sometimes different phones use send the information differently. You can edit how the information is read by you pc in "phyphox_reader.py"

