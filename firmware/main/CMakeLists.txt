# Edit following two lines to set component requirements (see docs)
idf_component_register(SRCS main.c rgb_led.c wifi_app.c http_server.c app_nvs.c wifi_reset_button.c dashboard.c dashboard_api.c climatesense.c
						INCLUDE_DIRS "."
						EMBED_FILES webpage/app.css webpage/app.js 
									webpage/favicon.ico webpage/icon.png 
									webpage/index.html 
									webpage/jquery-3.3.1.min.js
									webpage/interface/dashboard.html 
									webpage/interface/chart.js 
									webpage/interface/dashboard_charts.js
									webpage/interface/dashboard_sliders.js
									webpage/interface/dashboard_statusbar.js
									webpage/interface/dashboard.css
									webpage/interface/climatesense.html
									webpage/interface/climatesense.js
									webpage/interface/climatesense.css
									webpage/interface/navbar.css
									webpage/interface/slideswitch.css
									webpage/favicon.ico

									webpage/interface/home_occupid.png
									webpage/interface/home_unoccupid.png
									webpage/interface/motion_true.png
									webpage/interface/motion_false.png
									webpage/icon.png
									webpage/interface/RoomSense_Logo.png
						)
