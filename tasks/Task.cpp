/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include <base-logging/Logging.hpp>

using namespace simple_pose_integrator;

Task::Task(std::string const& name)
    : TaskBase(name)
    , body2odometry_fast(fast_transformer.registerTransformation("body", "odometry"))
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
    , body2odometry_fast(fast_transformer.registerTransformation("body", "odometry"))
{
}

Task::~Task()
{
}

void Task::pose_samples_inTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &pose_samples_in_sample)
{
    base::Affine3d new_body_in_odomerty;
    if(_body2odometry.get(ts, new_body_in_odomerty, true))
    {
        // save odometry pose at the time of the last received pose
        body_in_odomerty = new_body_in_odomerty;
        pose_sample = pose_samples_in_sample;
        _pose_samples.write(pose_samples_in_sample);
    }
    else
    {
        LOG_WARN_S << "Couldn't get " << _body_frame.value() << " in " << _odometry_frame.value() << " transformation! Keep last known pose offset.";
    }
}

void Task::fast_pose_callback(const base::Time& ts, const base::Time& timestamp)
{
    // return on old data
    if(timestamp <= pose_sample.time)
        return;

    if(!pose_sample.hasValidPosition() || !pose_sample.hasValidOrientation())
    {
        LOG_INFO_S << "Wait for valid pose sample!";
        return;
    }

    // apply newest odometry delta to last received pose
    base::Affine3d current_body_in_odomerty;
    if(body2odometry_fast.get(timestamp, current_body_in_odomerty))
    {
        base::samples::RigidBodyState new_pose_sample = pose_sample;
        new_pose_sample.time = timestamp;
        new_pose_sample.setTransform(pose_sample.getTransform() * (body_in_odomerty.inverse() * current_body_in_odomerty));
        _pose_samples.write(new_pose_sample);
    }
    else
    {
        LOG_ERROR_S << "Failed to receive " << _body_frame.value() << " in " << _odometry_frame.value() << ". This should never happen!";
        exception();
    }
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;

    fast_transformer.clear();
    fast_transformer.setTimeout( base::Time::fromSeconds(0.0) );

    std::vector<base::samples::RigidBodyState> const& staticTransforms = _static_transformations.value();
     for (size_t i = 0; i < staticTransforms.size(); ++i)
        fast_transformer.pushStaticTransformation(staticTransforms[i]);

    fast_transformer.setFrameMapping("body", _body_frame);
    fast_transformer.setFrameMapping("odometry", _odometry_frame);

    fast_pose_idx = fast_transformer.registerDataStream< ::base::Time >(
                            base::Time::fromSeconds(_body_in_odometry_period.value()),
                            boost::bind( &simple_pose_integrator::Task::fast_pose_callback, this, _1, _2), 1, "fast_pose");

    pose_sample.invalidate();

    return true;
}
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;
    return true;
}
void Task::updateHook()
{
    RTT::TaskContext::updateHook();

    // read new samples
    pullPorts();

    // read new dynamic transformations
    base::samples::RigidBodyState dynamicTransform;
    while(_dynamic_transformations.read(dynamicTransform, false) == RTT::NewData)
    {
        _transformer.pushDynamicTransformation(dynamicTransform);
        fast_transformer.pushDynamicTransformation(dynamicTransform);
        fast_transformer.pushData(fast_pose_idx, dynamicTransform.time, dynamicTransform.time);
    }

    // write out transfromer status
    do
    {
        const base::Time curTime(base::Time::now());
        if( curTime > _nextStatusTime )
        {
            _nextStatusTime = curTime + base::Time::fromSeconds( _transformer_status_period.value() );
            _transformer_stream_aligner_status.write(_transformer.getStatus());
            _transformer_status.write(_transformer.getTransformerStatus());
        }
    }
    // step sample aligned and not aligned transfromer
    while(_transformer.step());
    while(fast_transformer.step());
}
void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    TaskBase::stopHook();
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();

    fast_transformer.clear();
    fast_transformer.unregisterDataStream(fast_pose_idx);
}
