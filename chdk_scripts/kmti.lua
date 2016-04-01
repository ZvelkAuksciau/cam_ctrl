--[[
@title KMTI single shoot
@chdk_version 1.3
--]]

props=require("propcase")

set_prop(props.FLASH_MODE, 2)                                  -- disable built-in flash
set_prop(props.AF_ASSIST_BEAM,0)

sleep(1000)
press("left")
sleep(200)
release("left")
sleep(100)
press("right")
sleep(200)
release("right")
sleep(100)
press("set")
sleep(200)
release("set")

set_exit_key("no_key")
set_draw_title_line(0)

script_exit = false

print("Start sucesfull")

repeat
	wait_click(100)
	if is_key("shoot_full_only") then
		set_focus(100000)
		shoot()
	end
	if is_key("menu") then
		script_exit = true
	end
	if get_usb_power(1) == 1 then
		script_exit = true
	end
until (script_exit == true)