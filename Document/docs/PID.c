void pid_pos(void)
{
	if ((HAL_GetTick()-tick)>10)
		{
			//PIDControllerDirectionSet(&myPosPID, REVERSE);
			pulsecnt = __HAL_TIM_GET_COUNTER(&htim2);
			angleFeedback = (pulsecnt*360)/8000;
		  PIDSetpointSet(&myPosPID,dir_pos);
		  PIDInputSet(&myPosPID, angleFeedback);
		  PIDCompute(&myPosPID);
		  pidPositionOutput = PIDOutputGet(&myPosPID);
			pwmSendOut(pidPositionOutput);
			tick = HAL_GetTick();
		}
}
int PIDLientuc(int Output, int tocdodat, int tocdothuc)
{
	float outpid = 0;
	error1 = abs(tocdodat) - abs(tocdothuc);
	outpid = (Kp*error1) + (Kd*(error1 - last_error1));
	last_error1 = error1;
	if(Output + outpid >= 3599)
	{
		Output = 3599;
	}
	else if(Output + outpid <0)
	{
		Output = 0;
	}
	else
	{
		Output = Output + outpid;
	}
	return Output;
}
int PIDVel(float DesiredValue, float CurrentValue)
{
	static float err_p = 0;
	static float ui_p = 0;
	float err, up, ud, ui;
	int uout;
	
	err = DesiredValue - CurrentValue;
	
	up = Kp*err;
	ud = Kd*(err-err_p)/T;
	ui = ui_p+Ki*err*T;
	
	err_p = err;
	ui_p = ui;
	
	uout = (int)(up+ud+ui);
	if (uout > 3599) uout = 3599;
	else if (uout < 0) uout = 0;
	return uout;
}