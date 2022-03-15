# rSTATE

**rSTATE** is a ROS2 node designed to coordinate actions of a ROS2 network. Internally it is a state machine that allows for user definitions of network states and transitions. The state machine is largely designed to replicate the behaviour of [ROS2 managed lifecycle](https://design.ros2.org/articles/node_lifecycle.html) nodes on the network scale (with some extensions).

It offers 5 primary states:

- `Unconfigured`
- `Inactive`
- `Active`
- `Armed`
- `Finalized`

Transitions from primary states required external action, **rSTATE** cannot perform these transitions uncommanded.

There are 8 transition states:

- `Configuring`
- `Cleaning Up`
- `Activating`
- `Deactivating`
- `Arming`
- `Disarming`
- `Error Processing`

The transitions to and from these states are performed automatically when **rSTATE** is commanded with a valid transition.

There are 7 transitions available as commands for external processes:

- `Configure`
- `Clean Up`
- `Activate`
- `Deactivate`
- `Arm`
- `Disarm`
- `Shutdown`

These transitions are provided as goals in a [ROS2 action server](http://design.ros2.org/articles/actions.html).

```mermaid
stateDiagram-v2
    

    [*] --> Unconfigured
    Unconfigured --> Configuring
    Inactive --> CleaningUp
    Configuring --> Inactive : ✅
    Configuring --> Unconfigured : ✖️ 
    CleaningUp --> Unconfigured : ✅
    CleaningUp --> Inactive : ✖️

    Inactive --> Activating
    Activating --> Active : ✅
    Activating --> Inactive : ✖️
    Active --> Deactivating
    Deactivating --> Inactive : ✅
    Deactivating --> Active : ✖️

    Active --> Arming
    Arming --> Armed : ✅
    Arming --> Active : ✖️
    Armed --> Disarming
    Disarming --> Active : ✅
    Disarming --> Armed : ✖️

    ErrorProcessing --> ShuttingDown

    Unconfigured --> ShuttingDown
    Inactive --> ShuttingDown
    Active --> ShuttingDown
    Armed --> ShuttingDown
    ShuttingDown --> Finalized
    Finalized --> [*]


    state "Error Processing ❌" as ErrorProcessing

    state "Configuring ❗" as Configuring
    state "Cleaning Up ❗" as CleaningUp
    state "Activating ❗" as Activating
    state "Deactivating ❗" as Deactivating
    state "Arming ❗" as Arming
    state "Disarming ❗" as Disarming
    state "Shutting Down" as ShuttingDown
```
**NOTE:** Transitions state marked with ❗ have a possibility of failure. If performed successfully, the network will transition according to the path marked with ✅. **rSTATE** will attempt to recover from an error by returning the the network to the previous state. The network will enter the `Error Processing` state at this point. This behaviour is denoted by the symbol ✖️ on the state diagram. If it is not possible to restore the previous network state, the `Error Processing` transition will fail (denoted with ❌) and proceed to shut down the network.

## Primary State: Unconfigured

All managed lifecycle nodes controlled by **rSTATE** are `unconfigured`. No processes on the network are running.

## Primary State: Inactive

All managed lifecycle nodes controlled by **rSTATE** are `inactive`. No processes on the network are running.

## Primary State: Active

All managed lifecycle nodes controlled by **rSTATE** are `active`. The network is now allowed to execute intrinsically safe processes such as data logging. Nodes on the network must not advertise potentially unsafe actions such as valve actuation during this network state.

## Primary State: Armed

All managed lifecycle nodes controlled by **rSTATE** are `active`. The network is now allowed to execute all actions.

## Primary State: Finalized

All managed lifecycle nodes controlled by **rSTATE** are `finalized`. **rSTATE** will also finalize. From this state, the network must be restarted.

## Transition State: Configuring

**rSTATE** will `configure` all managed lifecycle nodes that it controls. If `cancel_goal` is recieved durring this transition, all nodes will be returned to `unconfigured`.

## Transition State: Cleaning Up

**rSTATE** will `cleanup` all managed lifecycle nodes that it controls. If `cancel_goal` is received during this transition, all nodes will be returned to `inactive`.

## Transition State: Activating

**rSTATE** will `activate` all managed lifecycle nodes that it controls. If `cancel_goal` is received during this transition, all nodes will be returned to `inactive`.

## Transition State: Deactivating

**rSTATE** will `deactivate` all managed lifecycle nodes that it controls. If `cancel_goal` is received during this transition, all nodes will be returned to `active`.

## Transition State: Arming

**rSTATE** will `arm` all nodes that it controls that have potentially unsafe actions. If `cancel_goal` is received during this transition, all nodes will be `disarmed`.

## Transition State: Disarming

**rSTATE** will `disarm` all nodes that it controls that have potentially unsafe actions. If `cancel_goal` is received during this transition, all nodes will be `armed`.

## Transition State: Shutting Down

**rSTATE** will `shutdown` all managed lifecycle nodes that it controls. Due to the nature of lifecycle nodes, this action cannot be cancelled.

## Transition State: Error Processing

**rSTATE** will attempt to return the network to the `unconfigured` state. If this is not possible the network will be `shutdown`.
