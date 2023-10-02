/*******************************************************************************
* Publish msgs (odometry, joint states, tf)
*******************************************************************************/
void publishDriveInformation(void)
{
  unsigned long time_now = millis();
  unsigned long step_time = time_now - prev_update_time;

  prev_update_time = time_now;
  ros::Time stamp_now = rosNow();
//
//  // calculate odometry
//  calcOdometry((double)(step_time * 0.001));
//
//  // odometry
//  updateOdometry();
//  odom.header.stamp = stamp_now;
//  odom_pub.publish(&odom);
//
//  // odometry tf
//  updateTF(odom_tf);
//  odom_tf.header.stamp = stamp_now;
//  tf_broadcaster.sendTransform(odom_tf);

  // joint states
dxl_joint_states_update(DXL_ID, joint_cnt);
real_joint_state.header.stamp = stamp_now;
real_joint_state_publisher.publish( &real_joint_state );
}
