{
    "ArduCopter": {
        "default_frame": "quad",
        "frames": {
            
            "+": {
                "waf_target": "bin/arducopter",
                "default_params_filename": "default_params/copter.parm",
            },
            "quad": {
                "model": "+",
                "waf_target": "bin/arducopter",
                "default_params_filename": "default_params/copter.parm",
            },
            "X": {
                "waf_target": "bin/arducopter",
                "default_params_filename": "default_params/copter.parm",
                
                
                
                "extra_mavlink_cmds": "param fetch frame; param set FRAME 1;",
            },
            "bfx": {
                "waf_target": "bin/arducopter",
                "default_params_filename": ["default_params/copter.parm",
                                            "default_params/copter-bfx.parm" ],
            },
            "djix": {
                "waf_target": "bin/arducopter",
                "default_params_filename": ["default_params/copter.parm",
                                            "default_params/copter-djix.parm" ],
            },
            "cwx": {
                "waf_target": "bin/arducopter",
                "default_params_filename": ["default_params/copter.parm",
                                            "default_params/copter-cwx.parm" ],
            },
            "hexa": {
                "waf_target": "bin/arducopter",
                "default_params_filename": ["default_params/copter.parm",
                                            "default_params/copter-hexa.parm" ],
            },
            "hexa-cwx": {
                "waf_target": "bin/arducopter",
                "default_params_filename": [
                    "default_params/copter.parm",
                    "default_params/copter-hexa.parm",
                    "default_params/copter-hexa-cwx.parm"
                ],
            },
            "hexa-dji": {
                "waf_target": "bin/arducopter",
                "default_params_filename": [
                    "default_params/copter.parm",
                    "default_params/copter-hexa.parm",
                    "default_params/copter-hexa-dji.parm"
                ],
            },
             "octa-cwx": {
                "waf_target": "bin/arducopter",
                "default_params_filename": [
                    "default_params/copter.parm",
                    "default_params/copter-octa.parm",
                    "default_params/copter-octa-cwx.parm"
                ],
            },
            "octa-quad-cwx": {
                "waf_target": "bin/arducopter",
                "default_params_filename": [
                    "default_params/copter.parm",
                    "default_params/copter-octaquad.parm",
                    "default_params/copter-octaquad-cwx.parm"
                ],
            },
            "octa-quad": {
                "waf_target": "bin/arducopter",
                "default_params_filename": ["default_params/copter.parm",
                                            "default_params/copter-octaquad.parm" ],
            },
            "octa": {
                "waf_target": "bin/arducopter",
                "default_params_filename": ["default_params/copter.parm",
                                            "default_params/copter-octa.parm" ],
            },
            "octa-dji": {
                "waf_target": "bin/arducopter",
                "default_params_filename": [
                    "default_params/copter.parm",
                    "default_params/copter-octa.parm",
                    "default_params/copter-octa-dji.parm"
                ],
            },
            "deca": {
                "waf_target": "bin/arducopter",
                "default_params_filename": ["default_params/copter.parm",
                                            "default_params/copter-deca.parm" ],
            },
            "deca-cwx": {
                "waf_target": "bin/arducopter",
                "default_params_filename": "default_params/copter.parm",
            },
            "tri": {
                "waf_target": "bin/arducopter",
                "default_params_filename": ["default_params/copter.parm",
                                            "default_params/copter-tri.parm" ],
            },
            "y6": {
                "waf_target": "bin/arducopter",
                "default_params_filename": ["default_params/copter.parm",
                                            "default_params/copter-y6.parm" ],
            },
            "dodeca-hexa": {
                "waf_target": "bin/arducopter",
                "default_params_filename": ["default_params/copter.parm",
                                            "default_params/copter-dodecahexa.parm" ],
            },
            
            "IrisRos": {
                "waf_target": "bin/arducopter",
                "default_params_filename": "default_params/copter.parm",
                "external": "True",
            },
            "gazebo-iris": {
                "waf_target": "bin/arducopter",
                "default_params_filename": ["default_params/copter.parm",
                                            "default_params/gazebo-iris.parm"],
                "external": "True",
            },
            "airsim-copter": {
                "waf_target": "bin/arducopter",
                "default_params_filename": ["default_params/copter.parm",
                                            "default_params/airsim-quadX.parm"],
                "external": "True",
            },
            
            "heli": {
                "waf_target": "bin/arducopter-heli",
                "default_params_filename": "default_params/copter-heli.parm",
            },
            "heli-dual": {
                "waf_target": "bin/arducopter-heli",
                "default_params_filename": ["default_params/copter-heli.parm",
                                            "default_params/copter-heli-dual.parm"],
            },
            "singlecopter": {
                "waf_target": "bin/arducopter",
                "default_params_filename": "default_params/copter-single.parm",
            },
            "coaxcopter": {
                "waf_target": "bin/arducopter",
                "default_params_filename": ["default_params/copter-single.parm",
                                            "default_params/copter-coax.parm"],
            },
            "scrimmage-copter" : {
                "waf_target": "bin/arducopter",
                "default_params_filename": "default_params/copter.parm",
                "external": "True",
            },
            "calibration": {
                "extra_mavlink_cmds": "module load sitl_calibration;",
                "external": "True",  
            },
            "Callisto": {
                "model": "octa-quad:@ROMFS/models/Callisto.json",
                "waf_target": "bin/arducopter",
                "default_params_filename": ["default_params/copter.parm",
                                            "models/Callisto.param"],
            },
        },
    },
    "Helicopter": {
        "default_frame": "heli",
        "frames": {
            "heli": {
                "waf_target": "bin/arducopter-heli",
                "default_params_filename": "default_params/copter-heli.parm",
            },
            "heli-dual": {
                "waf_target": "bin/arducopter-heli",
                "default_params_filename": ["default_params/copter-heli.parm",
                                            "default_params/copter-heli-dual.parm"],
            },
            
            
            
            
            
        },
    },
    "Blimp": {
        "default_frame": "quad",
        "frames": {
            
            "quad": {
                "model": "+",
                "waf_target": "bin/blimp",
                "default_params_filename": "default_params/blimp.parm",
            },
        },
    },
    "ArduPlane": {
        "default_frame": "plane",
        "frames": {
            
            "quadplane-tilttri": {
                "waf_target": "bin/arduplane",
                "default_params_filename": "default_params/quadplane-tilttri.parm",
            },
            "quadplane-tilttrivec": {
                "waf_target": "bin/arduplane",
                "default_params_filename": "default_params/quadplane-tilttrivec.parm",
            },
            "quadplane-tilthvec": {
                "waf_target": "bin/arduplane",
                "default_params_filename": ["default_params/plane.parm", "default_params/quadplane-tilthvec.parm"],
            },
            "quadplane-tri": {
                "waf_target": "bin/arduplane",
                "default_params_filename": "default_params/quadplane-tri.parm",
            },
            "quadplane-cl84" : {
                "waf_target" : "bin/arduplane",
                "default_params_filename": "default_params/quadplane-cl84.parm",
            },
            "quadplane": {
                "waf_target": "bin/arduplane",
                "default_params_filename": "default_params/quadplane.parm",
            },
            "firefly": {
                "waf_target": "bin/arduplane",
                "default_params_filename": "default_params/firefly.parm",
            },
            "plane-elevon": {
                "waf_target": "bin/arduplane",
                "default_params_filename": ["default_params/plane.parm", "default_params/plane-elevons.parm"],
            },
            "plane-vtail": {
                "waf_target": "bin/arduplane",
                "default_params_filename": ["default_params/plane.parm", "default_params/plane-vtail.parm"],
            },
            "plane-tailsitter": {
                "waf_target": "bin/arduplane",
                "default_params_filename": "default_params/plane-tailsitter.parm",
            },
            "plane-jet": {
                "waf_target": "bin/arduplane",
                "default_params_filename": ["default_params/plane.parm", "default_params/plane-jet.parm"],
            },
            "plane": {
                "waf_target": "bin/arduplane",
                "default_params_filename": "default_params/plane.parm",
            },
            "plane-dspoilers": {
                "waf_target": "bin/arduplane",
                "default_params_filename": ["default_params/plane.parm", "default_params/plane-dspoilers.parm"]
            },
            "plane-soaring": {
                "waf_target": "bin/arduplane",
                "default_params_filename": ["default_params/plane.parm", "default_params/plane-soaring.parm"]
            },
            "gazebo-zephyr": {
                "waf_target": "bin/arduplane",
                "default_params_filename": "default_params/gazebo-zephyr.parm",
            },
            "last_letter": {
                "waf_target": "bin/arduplane",
            },
            "CRRCSim": {
                "waf_target": "bin/arduplane",
            },
            "jsbsim": {
                "waf_target": "bin/arduplane",
                "default_params_filename": "default_params/plane-jsbsim.parm",
            },
            "scrimmage-plane" : {
                "waf_target": "bin/arduplane",
                "default_params_filename": "default_params/plane.parm",
            },
            "calibration": {
                "extra_mavlink_cmds": "module load sitl_calibration;",
            },
        },
    },
    "Rover": {
        "default_frame": "rover",
        "frames": {
            
            "rover": {
                "waf_target": "bin/ardurover",
                "default_params_filename": "default_params/rover.parm",
            },
            "rover-skid": {
                "waf_target": "bin/ardurover",
                "default_params_filename": ["default_params/rover.parm",
                                            "default_params/rover-skid.parm"],
            },
            "rover-vectored": {
                "waf_target": "bin/ardurover",
                "default_params_filename": ["default_params/rover.parm",
                                            "default_params/rover-vectored.parm"],
            },
            "balancebot": {
                "waf_target": "bin/ardurover",
                "default_params_filename": ["default_params/rover.parm",
                                            "default_params/rover-skid.parm",
                                            "default_params/balancebot.parm"],
            },
            "sailboat": {
                "waf_target": "bin/ardurover",
                "default_params_filename": ["default_params/rover.parm",
                                            "default_params/sailboat.parm"],
            },
            "sailboat-motor": {
                "waf_target": "bin/ardurover",
                "default_params_filename": ["default_params/rover.parm",
                                            "default_params/sailboat-motor.parm"],
            },
            "gazebo-rover": {
                "waf_target": "bin/ardurover",
                "default_params_filename": ["default_params/rover.parm",
                                            "default_params/rover-skid.parm"],
            },
            "airsim-rover": {
                "waf_target": "bin/ardurover",
                "default_params_filename": ["default_params/rover.parm",
                                            "default_params/airsim-rover.parm"],
            },
            "calibration": {
                "extra_mavlink_cmds": "module load sitl_calibration;",
            },
        },
    },
    "ArduSub": {
        "default_frame": "vectored",
        "frames": {
            "vectored": {
                "waf_target": "bin/ardusub",
                "default_params_filename": "default_params/sub.parm",
            },
            "vectored_6dof": {
                "waf_target": "bin/ardusub",
                "default_params_filename": "default_params/sub-6dof.parm",
            },
            "gazebo-bluerov2": {
                "waf_target": "bin/ardusub",
                "default_params_filename": "default_params/sub.parm",
            },
        },
    },
    "AntennaTracker": {
        "default_frame": "tracker",
        "frames": {
            "tracker": {
                "waf_target": "bin/antennatracker",
            },
        },
    },
}