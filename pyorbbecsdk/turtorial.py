from pyorbbecsdk import *

pipeline = Pipeline()
pipeline.start()                        # uses default config from OrbbecSDKConfig.xml
frames = pipeline.wait_for_frames(1000) # get synchronized Color + Depth frames