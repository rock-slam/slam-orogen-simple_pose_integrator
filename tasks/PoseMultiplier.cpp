/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "PoseMultiplier.hpp"

using namespace simple_pose_integrator;

PoseMultiplier::PoseMultiplier(std::string const& name)
    : PoseMultiplierBase(name)
{
    drift.initUnknown();

}

PoseMultiplier::~PoseMultiplier()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See PoseMultiplier.hpp for more detailed
// documentation about them.

bool PoseMultiplier::configureHook()
{
    if (! PoseMultiplierBase::configureHook())
        return false;
    return true;
}
bool PoseMultiplier::startHook()
{
    if (! PoseMultiplierBase::startHook())
        return false;
    return true;
}
void PoseMultiplier::updateHook()
{
    PoseMultiplierBase::updateHook();

    _drift.readNewest(drift);
    if (_pose_updates.readNewest(pose) == RTT::NewData) {

        // if (drift.sourceFrame != pose.targetFrame) {
        //     throw std::runtime_error("input frames have to match for a valid result");
        // }

        base::samples::RigidBodyState new_pose_sample = pose;
        new_pose_sample.sourceFrame = _source_frame.value();
        new_pose_sample.targetFrame = _target_frame.value();
        new_pose_sample.setTransform(drift.getTransform() * pose.getTransform());

        _pose_samples.write(new_pose_sample);
    }
}
void PoseMultiplier::errorHook()
{
    PoseMultiplierBase::errorHook();
}
void PoseMultiplier::stopHook()
{
    PoseMultiplierBase::stopHook();
}
void PoseMultiplier::cleanupHook()
{
    PoseMultiplierBase::cleanupHook();
}
