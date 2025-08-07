This package needs to be installed inside the isaacsim.example.interactive

Then, the toml file from that isaacsim extension needs to be modified to include this package: 

[[python.module]]
name = "isaacsim.examples.interactive.brownbot_policy"

Also, in order to make this package works, the isaacsim.robot.policy.examples needs to be modified with the content of the
./data_temp/policy_controller copy.py