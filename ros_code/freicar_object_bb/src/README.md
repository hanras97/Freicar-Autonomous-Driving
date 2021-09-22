# Setup

Install all the python dependencies using pip:

```console
pip3 install -r requirements.txt
```

# Dataset

The dataset will be downloaded by calling the following commands

```console
cd dataloader
python3 run_freicar_dataloader.py
```

This downloads a `.zip` file and extracts it content in the `dataloader` directory.
After extraction is completed, the zipfile will be automatically removed.

# Training

First, in a separate console window, start a visdom server

```console
python3 -m visdom.server
```
With a browser, you can navigate to `http://localhost:8097` to look at your training stats.
If you see no data but your training is already running, select the `FreiCar Object Detection` environment 
at the top of the visdom webpage.




Now, in a different console window, you can start your training

```console
python3 train.py -c 0 -p freicar-detection --batch_size 8 --lr 1e-5
```
Change the batch size in case the model does not fit into GPU memory.


    


# Inference Script 


If you want to have a look at your model predictions, you can run
```console
python3 inference.py -w ./logs/freicar-detection/efficientdet-d0_99_109100.pth
```


# Evaluation Script 

For evaluating the model and calculating the mean average precision, run:
```console
python3 evaluate.py -w ./logs/freicar-detection/efficientdet-d0_99_109100.pth
```

# ROS Node
To initiate ROS node, start the core
```console
roscore
```
Next, launch the freicar simulator using
```console
roslaunch freicar_launch local_comp_launch.launch
```
Next, run the agent node using
```console
roslaunch freicar_agent sim_agent.launch name:=freicar_1 tf_name:=freicar_1 spawn/x:=0 spawn/y:=0 spawn/z:=0 spawn/heading:=20 use_yaml_spawn:=true sync_topic:=!
```
Now, to publishing the bounding boxes to a rostopic
```console
python3 rospubsub.py
```
The bounding boxes data can be inferred
```console
rostopic echo /freicar_1/bounding_box
```