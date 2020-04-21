bool check(hms_client::ping_pong::Request  &req,
         hms_client::ping_pong::Response &res)
{
  res.msg.header.stamp = ros::Time::now();
  res.health = 1;
  if(obstacle_flag)//assuming this variable is accesible otherwise change accordingly
    res.error_code = 1;
  else
    res.error_code = 0;
  return true;
}
//the next line in main
ros::ServiceServer service = n.advertiseService("health_check_obstacle_2d", check);