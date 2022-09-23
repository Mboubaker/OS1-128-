Before we begin, let’s take a quick look at the unique features of Ouster lidar sensor data that makes this project possible. 
Understanding Ouster’s rich data layers and its perfect 1:1 spatial correspondence of points is the key to understanding how 
we are able to combine 2D algorithms and 3D spatial data.


###Ouster Data Layer : 

OS1-128 lidar sensor outputs four data layers for each pixel: Range, Signal, Near-IR, and Reflectivity. 

⇒ Range: The distance of the point from the sensor origin, calculated by using the time of flight of the laser pulse

⇒ Signal: The strength of the light returned to the sensor for a given point. Signal for Ouster digital lidar sensors 
is expressed in the number of photons of light detected

⇒ Near-IR: The strength of sunlight collected for a given point, also expressed in the quantity of photons detected 
that was not produced by the sensor’s laser pulse

⇒ Reflectivity: The reflectivity of the surface (or object) that was detected by the sensor.



###Acquisition of data layers using Ouster Python SDK and OS1-128 

Our lidar sensor provides two files: a PCAP file which is raw UDP packets captured by the sensor, and a JSON file which contains 
the sensor’s metadata that’s required to interpret packets. Using the SDK, we first loaded the sensor metadata using the client module.
       from ouster import client
       metadata_path = '<DATA_JSON_PATH>'
       with open(metadata_path, 'r') as f:
           metadata = client.SensorInfo(f.read())
           
With the metadata, we can now read the PCAP file by instantiating pcap.Pcap using the pcap module:   
      from ouster import pcap
      pcap_path = '<DATA_PCAP_PATH>' 
      pcap_file = pcap.Pcap(pcap_path, metadata)
      
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

###Test and results 



<p align="center">   
  <img src="https://user-images.githubusercontent.com/97898968/158196193-bf24c67f-d13d-409f-9310-3b075e288b1d.png?raw=true" alt="Sublime's custom image"/>
</p>
           
