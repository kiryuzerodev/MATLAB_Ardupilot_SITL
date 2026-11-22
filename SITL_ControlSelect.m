function pwm_out = SITL_ControlSelect(pwm_ap, pwm_custom, armed, mode, params)
% Decides who gets to fly each channel: ArduPilot or you.
% MANUAL mode always goes back to ArduPilot so the Lua doublets/3211s
% can do their thing without your controller elbowing them out.
pwm_out = pwm_ap;
if armed == 1 && mode ~= 0
    for k = 1:length(params.CTRL_MASK)
        ch = params.CTRL_MASK(k);
        pwm_out(ch) = pwm_custom(ch);
    end
end
end