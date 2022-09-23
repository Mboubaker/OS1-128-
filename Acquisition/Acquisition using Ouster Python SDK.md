# Ouster Data pca
Before we begin, let’s take a quick look at the unique features of Ouster lidar sensor data that makes this project possible. 
Understanding Ouster’s rich data layers and its perfect 1:1 spatial correspondence of points is the key to understanding how 
we are able to combine 2D algorithms and 3D spatial data.


## Ouster Data Layer : 

OS1-128 lidar sensor outputs four data layers for each pixel: Range, Signal, Near-IR, and Reflectivity. 

⇒ Range: The distance of the point from the sensor origin, calculated by using the time of flight of the laser pulse

⇒ Signal: The strength of the light returned to the sensor for a given point. Signal for Ouster digital lidar sensors 
is expressed in the number of photons of light detected

⇒ Near-IR: The strength of sunlight collected for a given point, also expressed in the quantity of photons detected 
that was not produced by the sensor’s laser pulse

⇒ Reflectivity: The reflectivity of the surface (or object) that was detected by the sensor.


## Ouster Python SDK
The Ouster Sensor SDK provides developers interfaces for interacting with sensor hardware.
The SDK includes APIs for : 

-  Querying and setting sensor configuration.

-  Recording and reading data in pcap format.

-  Reading and buffering sensor UDP data streams reliably.

-  Conversion of raw data to range/signal/near_ir/reflectivity images (destaggering).

-  Efficient projection of range measurements to Cartesian (x, y, z) corrdinates.

-  Visualization of multi-beam flash lidar data.

### Ouster SDK Installation :
please follow this link https://static.ouster.dev/sdk-docs/installation.html

## Acquisition of data layers using Ouster Python SDK and OS1-128 

the OS1-128 lidar sensor provides two files: a PCAP file which is raw UDP packets captured by the sensor, and a JSON file which contains 
the sensor’s metadata that’s required to interpret packets. Using the SDK, we first loaded the sensor metadata using the client module.
       from ouster import client
       metadata_path = '<DATA_JSON_PATH>'
       with open(metadata_path, 'r') as f:
           metadata = client.SensorInfo(f.read())
           
With the metadata, we can now read the PCAP file by instantiating pcap.Pcap using the pcap module:   
      from ouster import pcap
      pcap_path = '<DATA_PCAP_PATH>' 
      pcap_file = pcap.Pcap(pcap_path, metadata)

### Saving data layer as grayscale image :       

Now, let’s parse the PCAP into individual scans using the Scans module. A scan is a restructured 360º view of data. 
Since we’ll be using the reflectivity and range layers,let’s pull them out from each scan. 
We collected multiple PCAP files with an OS0-128 (you can see how to do it here), and then used the Ouster Python SDK to pull
the reflectivity, intensity, range and near_IR layers of each scan and utilized OpenCV’s cv2.imwrite function to save the layers
as grayscale images for data labeling. The calls look like the following:
       import cv2
       import numpy as np
       from contextlib import closing
       from ouster import client
       from ouster import pcap

       with open(metadata_path, 'r') as f:
           metadata = client.SensorInfo(f.read())

       source = pcap.Pcap(pcap_path, metadata)

       counter = 0
       with closing(client.Scans(source)) as scans:
           for scan in scans:
               counter += 1
               ref_field = scan.field(client.ChanField.REFLECTIVITY)
               ref_val = client.destagger(source.metadata, ref_field)
               ref_img = ref_val.astype(np.uint8)

               filename = 'extract'+str(counter)+'.jpg'
               cv2.imwrite(img_path+filename, ref_img)
### Saving data layer as csv file : 

By analyzing the PCAP, we extracted all scans using "the Scans module" in Ouster python SDK .
Since we used the Intensity and Range layers, we ended up saving the intensity and range of each pixel in the scan to a csv file.
The calls look like the following:

      import numpy as np
      np.savetxt('./Intensity_csv/Intensity'+ str(counter) + '.csv', ref_field, delimiter=' ')
      
## Test and results 

### layer of intensity : 

<p align="center">   
  <img src="https://user-images.githubusercontent.com/97898968/191918297-c0996643-6e5e-40b7-a92c-76d3b6ad1b75.png?raw=true" alt="Sublime's custom image"/>
       
</p>
<p align="center">                                  
Figure : the intensity layer as grayscale image
</p>
                                 
 <p align="center">   
  <img src="https://user-images.githubusercontent.com/97898968/191921791-4ad8f81f-6265-417e-af3d-fd3b3d662729.png?raw=true" alt="Sublime's custom image"/>
</p>                                
 <p align="center">                                 
  Figure : the intensity layer as csv file
 </p>                               

### layer of reflectivity :  

<p align="center">   
  <img src="https://user-images.githubusercontent.com/97898968/191918566-8a9bb9e4-b2c1-405d-9a27-bd38623d996f.png?raw=true" alt="Sublime's custom image"/>
</p>
<p align="center">   
 Figure : the reflectivity layer as grayscale image
 </p>
                               
                                 
                                 
                                 
  <p align="center">   
  <img src="https://user-images.githubusercontent.com/97898968/191921776-39afb2f0-cb11-460e-92eb-fb7451e50978.png?raw=true" alt="Sublime's custom image"/>
</p>                               
 
 <p align="center">                      
Figure : the reflectivity layer as csv file 
 </p>                               
      
