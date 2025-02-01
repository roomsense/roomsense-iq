/**
 * Add gobals here
 */
var seconds 	= null;
var otaTimerVar =  null;
var wifiConnectInterval = null;
var mqtt_connect_interval = null;
var ap_key_interval = null;

/**
 * Initialize functions here.
 */
$(document).ready(function(){
	var sw2 = document.getElementById('switch-4'); 
  
	var c2i_state = localStorage.getItem("c2i_state");
	if(c2i_state!=null){
		console.log("got C2Istate : ",c2i_state);
		sw2.checked = (c2i_state == "true") ? true : false;
	}

			sw2.addEventListener('change', function() {
				c2i_state = sw2.checked?"true":"false";
				localStorage.setItem("c2i_state",c2i_state);
				console.log("Saving state ",c2i_state);
			});
	getUpdateStatus();
	getConnectInfo();
	mqtt_get_connect_info();
	
	$("#connect_wifi").on("click", function(){
		checkCredentials();
	}); 
	$("#disconnect_wifi").on("click", function(){
		disconnectWifi();
	}); 
	$("#connect_mqtt").on("click", function(){
	    checkCredentials_mqtt();
	}); 
	$("#update_ap_key").on("click", function(){
	    // Clear the messages before performing the check
	    $("#ap_key_errors").html("");
	    $("#ap_save_status").html("");
		checkKey();
	}); 
});   

/**
 * Gets file name and size for display on the web page.
 */        
function getFileInfo() 
{
    var x = document.getElementById("selected_file");
    var file = x.files[0];

    document.getElementById("file_info").innerHTML = "<h4>File: " + file.name + "<br>" + "Size: " + file.size + " bytes</h4>";
}

/**
 * Handles the firmware update.
 */
function updateFirmware() 
{
    // Form Data
    var formData = new FormData();
    var fileSelect = document.getElementById("selected_file");
    
    if (fileSelect.files && fileSelect.files.length == 1) 
	{
        var file = fileSelect.files[0];
        formData.set("file", file, file.name);
        document.getElementById("ota_update_status").innerHTML = "Uploading " + file.name + ", Firmware Update in Progress...";

        // Http Request
        var request = new XMLHttpRequest();

        request.upload.addEventListener("progress", updateProgress);
        request.open('POST', "/OTAupdate");
        request.responseType = "blob";
        request.send(formData);
    } 
	else 
	{
        window.alert('Select A File First')
    }
}

/**
 * Progress on transfers from the server to the client (downloads).
 */
function updateProgress(oEvent) 
{
    if (oEvent.lengthComputable) 
	{
        getUpdateStatus();
    } 
	else 
	{
        window.alert('total size is unknown')
    }
}

/**
 * Posts the firmware udpate status.
 */
function getUpdateStatus() 
{
    var xhr = new XMLHttpRequest();
    var requestURL = "/OTAstatus";
    xhr.open('POST', requestURL, false);
    xhr.send('ota_update_status');

    if (xhr.readyState == 4 && xhr.status == 200) 
	{		
        var response = JSON.parse(xhr.responseText);
						
	 	document.getElementById("latest_firmware").innerHTML = response.compile_date + " - " + response.compile_time

		// If flashing was complete it will return a 1, else -1
		// A return of 0 is just for information on the Latest Firmware request
        if (response.ota_update_status == 1) 
		{
    		// Set the countdown timer time
            seconds = 10;
            // Start the countdown timer
            otaRebootTimer();
        } 
        else if (response.ota_update_status == -1)
		{
            document.getElementById("ota_update_status").innerHTML = "!!! Upload Error !!!";
        }
    }
}

/**
 * Displays the reboot countdown.
 */
function otaRebootTimer() 
{	
    document.getElementById("ota_update_status").innerHTML = "OTA Firmware Update Complete. This page will close shortly, Rebooting in: " + seconds;

    if (--seconds == 0) 
	{
        clearTimeout(otaTimerVar);
        window.location.reload();
    } 
	else 
	{
        otaTimerVar = setTimeout(otaRebootTimer, 1000);
    }
}

/**
 * Clears the connection status interval.
 */
function stopWifiConnectStatusInterval()
{
	if (wifiConnectInterval != null)
	{
		clearInterval(wifiConnectInterval);
		wifiConnectInterval = null;
	}
}

/**
 * Gets the WiFi connection status.
 */
function getWifiConnectStatus()
{
	var xhr = new XMLHttpRequest();
	var requestURL = "/wifiConnectStatus";
	xhr.open('POST', requestURL, true);
	xhr.send('wifi_connect_status');

	xhr.onreadystatechange = function () {
		if (xhr.readyState == 4 && xhr.status == 200)
		{
			var response = JSON.parse(xhr.responseText);
			
			document.getElementById("wifi_connect_status").innerHTML = "Connecting...";
			
			if (response.wifi_connect_status == 2)
			{
				document.getElementById("wifi_connect_status").innerHTML = "<h4 class='rd'>Failed to Connect. Please check your AP credentials and compatibility</h4>";
				stopWifiConnectStatusInterval();
			}
			else if (response.wifi_connect_status == 3)
			{
				document.getElementById("wifi_connect_status").innerHTML = "<h4 class='gr'>Connection Success!</h4>";
				stopWifiConnectStatusInterval();
				getConnectInfo();
			}
		}
	};
}

/**
 * Starts the interval for checking the connection status.
 */
function startWifiConnectStatusInterval()
{
	wifiConnectInterval = setInterval(getWifiConnectStatus, 2800);
}

/**
 * Connect WiFi function called using the SSID and password entered into the text fields.
 */
function connectWifi()
{
	// Get the SSID and password
	selectedSSID = $("#connect_ssid").val();
	pwd = $("#connect_pass").val();
	
	$.ajax({
		url: '/wifiConnect.json',
		dataType: 'json',
		method: 'POST',
		cache: false,
		headers: {'my-connect-ssid': selectedSSID, 'my-connect-pwd': pwd},
		data: {'timestamp': Date.now()}
	});
	
	startWifiConnectStatusInterval();
}

/**
 * Checks credentials on connect_wifi button click.
 */
function checkCredentials()
{
	errorList = "";
	credsOk = true;
	
	selectedSSID = $("#connect_ssid").val();
	pwd = $("#connect_pass").val();
	
	if (selectedSSID == "")
	{
		errorList += "<h4 class='rd'>SSID cannot be empty!</h4>";
		credsOk = false;
	}
	if (pwd == "")
	{
		errorList += "<h4 class='rd'>Password cannot be empty!</h4>";
		credsOk = false;
	}
	
	if (credsOk == false)
	{
		$("#wifi_connect_credentials_errors").html(errorList);
	}
	else
	{
		$("#wifi_connect_credentials_errors").html("");
		connectWifi();    
	}
}

/**
 * Shows the WiFi password if the box is checked.
 */
function showPassword()
{
	var x = document.getElementById("connect_pass");
	if (x.type === "password")
	{
		x.type = "text";
	}
	else
	{
		x.type = "password";
	}
}

/**
 * Gets the connection information for displaying on the web page.
 */
function getConnectInfo()
{
	$.getJSON('/wifiConnectInfo.json', function(data)
	{
		$("#connected_ap_label").html("Connected to: ");
		$("#connected_ap").text(data["ap"]);
		
		$("#ip_address_label").html("IP Address: ");
		$("#wifi_connect_ip").text(data["ip"]);
		
		$("#netmask_label").html("Netmask: ");
		$("#wifi_connect_netmask").text(data["netmask"]);
		
		$("#gateway_label").html("Gateway: ");
		$("#wifi_connect_gw").text(data["gw"]);
		
		document.getElementById('disconnect_wifi').style.display = 'block';
	});
}

/**
 * Disconnects Wifi once the disconnect button is pressed and reloads the web page.
 */
function disconnectWifi()
{
	$.ajax({
		url: '/wifiDisconnect.json',
		dataType: 'json',
		method: 'DELETE',
		cache: false,
		data: { 'timestamp': Date.now() }
	});
	// Update the web page
	setTimeout("location.reload(true);", 2000);
}


/* ----------------------------------MQTT functions --------------------*/
/**
 * Clears the connection status interval.
 */
function stop_mqtt_connect_status_interval()
{
	if (mqtt_connect_interval != null)
	{
		clearInterval(mqtt_connect_interval);
		mqtt_connect_interval = null;
	}
}

/**
 * Gets the get_mqtt_connect_status.
 */
function get_mqtt_connect_status()
{
	var xhr = new XMLHttpRequest();
	var requestURL = "/mqtt_connect_status";
	xhr.open('POST', requestURL, true);
	xhr.send('mqtt_connect_status');
	xhr.onreadystatechange = function () {
		if (xhr.readyState == 4 && xhr.status == 200)
		{
			var response = JSON.parse(xhr.responseText);
			
			document.getElementById("mqtt_connect_status").innerHTML = "Connecting...";
			
			if (response.mqtt_connect_status == 0)
			{
				document.getElementById("mqtt_connect_status").innerHTML = "<h4 class='rd'> Connecting..</h4>";
				stop_mqtt_connect_status_interval();
			}
			
			else if (response.mqtt_connect_status == 1)
			{
				document.getElementById("mqtt_connect_status").innerHTML = "<h4 class='gr'> Connecting...</h4>";
				//stop_mqtt_connect_status_interval();
			}
			
			else if (response.mqtt_connect_status == 2)
			{
				document.getElementById("mqtt_connect_status").innerHTML = "<h4 class='rd'>Failed to Connect. Please check your MQTT broker credentials and compatibility.</h4>";
				mqtt_get_connect_info();
				//stop_mqtt_connect_status_interval();
			}
			else if (response.mqtt_connect_status == 3)
			{
				document.getElementById("mqtt_connect_status").innerHTML = "<h4 class='gr'>Connection Success!</h4>";
				stop_mqtt_connect_status_interval();
				mqtt_get_connect_info();
			}
			else if (response.mqtt_connect_status == 4)
			{
				document.getElementById("mqtt_connect_status").innerHTML = "<h4 class='rd'>Reconnecting...</h4>";
				//stop_mqtt_connect_status_interval();
			}
			else if (response.mqtt_connect_status == 5)
			{
				document.getElementById("mqtt_connect_status").innerHTML = "<h4 class='gr'>Already Connected</h4>";
				stop_mqtt_connect_status_interval();
				mqtt_get_connect_info();
			}
		}
	};	
}

/**
 * Starts the interval for checking the connection status.
 */
function start_mqtt_connect_status_interval()
{
	mqtt_connect_interval = setInterval(get_mqtt_connect_status, 1000);
}

/**
 * connect_mqtt function called using the username and password entered into the text fields.
 */
function connect_mqtt()
{
	// Get the username and password
	selected_host = $("#mqtt_host").val();
	selected_port = $("#mqtt_port").val();
	selected_username = $("#mqtt_username").val();
	pwd = $("#mqtt_pass").val();
	
	$.ajax({
		url: '/mqtt_connect.json',
		dataType: 'json',
		method: 'POST',
		cache: false,
		headers: {'my-connect-host': selected_host, 'my-connect-port': selected_port, 'my-connect-username': selected_username, 'my-connect-mqtt-pwd': pwd},
		data: {'timestamp': Date.now()}
	});
	
	start_mqtt_connect_status_interval();
}

/**
 * Checks credentials on connect_mqtt button click.
 */
function checkCredentials_mqtt()
{
	errorList = "";
	credsOk = true;
	
	
	selected_host = $("#mqtt_host").val();
	ipRegex = /^(\d{1,3}\.){3}\d{1,3}$/;
	
	selected_port = $("#mqtt_port").val();
	
	selected_username = $("#mqtt_username").val();
	mqtt_pwd = $("#mqtt_pass").val();
	
	if (selected_host == "")
	{
		errorList += "<h4 class='rd'>Host cannot be empty!</h4>";
		credsOk = false;
	}
	
	if (!ipRegex.test(selected_host)) 
	{
        errorList += "<h4 class='rd'>Invalid Host! Expected Format XXX.XXX.XXX.XXX</h4>";
		credsOk = false;
    }
        
    if (selected_port == "")
	{
		errorList += "<h4 class='rd'>Port number cannot be empty!</h4>";
		credsOk = false;
	}
	
	if (isNaN(selected_port))
	{
        errorList += "<h4 class='rd'>Invalid Port! Expected a number.</h4>";
		credsOk = false;
    }
	
	if (selected_username == "")
	{
		errorList += "<h4 class='rd'>Username cannot be empty!</h4>";
		credsOk = false;
	}
	if (mqtt_pwd == "")
	{
		errorList += "<h4 class='rd'>Password cannot be empty!</h4>";
		credsOk = false;
	}
	
	if (credsOk == false)
	{
		$("#mqtt_connect_credentials_errors").html(errorList);
	}
	else
	{
		$("#mqtt_connect_credentials_errors").html("");
		connect_mqtt();    
	}
}

/**
 * Shows the MQTT password if the box is checked.
 */
function mqtt_show_password()
{
	var x = document.getElementById("mqtt_pass");
	if (x.type === "password")
	{
		x.type = "text";
	}
	else
	{
		x.type = "password";
	}
}

/**
 * Gets the connection information for displaying on the web page.
 */
function mqtt_get_connect_info()
{
	$.getJSON('/mqtt_connect_info.json', function(data)
	{
		$("#connected_user_label").html("Connected to: ");
		$("#connected_user").text(data["user"]);
		
		$("#host_ip_label").html("Host: ");
		$("#mqtt_connect_ip").text(data["host_ip"]);
		
		
		$("#port_label").html("Port: ");
		$("#mqtt_connect_port").text(data["port"]);
		
	});
}

/////////////////////////Access point key

/**
 * Clears the connection status interval.
 */
function stop_ap_key_status_interval()
{
	if (ap_key_interval != null)
	{
		clearInterval(ap_key_interval);
		ap_key_interval = null;
	}
}

/**
 * Gets the WiFi connection status.
 */
function get_ap_key_save_status()
{
	var xhr = new XMLHttpRequest();
	var requestURL = "/apKeyStatus";
	xhr.open('POST', requestURL, true);
	xhr.send('ap_save_status');

	xhr.onreadystatechange = function () {
		if (xhr.readyState == 4 && xhr.status == 200)
		{
			var response = JSON.parse(xhr.responseText);
			
			document.getElementById("ap_save_status").innerHTML = "Updating...";
			
			if (response.ap_save_status == 2)
			{
				document.getElementById("ap_save_status").innerHTML = "<h4 class='rd'>Failed to Update access Point Key</h4>";
				stop_ap_key_status_interval();
			}
			else if (response.ap_save_status == 1)
			{
				document.getElementById("ap_save_status").innerHTML = "<h4 class='gr'>Access Point Key Updated Successfully</h4>";
				stop_ap_key_status_interval();
			}
		}
	};
}

/**
 * Starts the interval for checking the connection status.
 */
function start_ap_key_status_interval()
{
	ap_key_interval = setInterval(get_ap_key_save_status, 1000);
}


function set_ap_key()
{
	// Get the key
	selectedKEY = $("#update_key").val();
	
	$.ajax({
		url: '/key.json',
		dataType: 'json',
		method: 'POST',
		cache: false,
		headers: {'my-key': selectedKEY},
		data: {'timestamp': Date.now()}
	});
	start_ap_key_status_interval();
}

/**
 * Checks credentials on connect_wifi button click.
 */
function checkKey()
{
	errorList = "";
	credsOk = true;
	
	selectedKEY = $("#update_key").val();
	
	if (selectedKEY == "")
	{
		errorList += "<h4 class='rd'>Key cannot be empty!</h4>";
		credsOk = false;
	}
	
    else if (selectedKEY.length < 8 || selectedKEY.length > 32)
    {
            errorList += "<h4 class='rd'>Key must be between 8 and 32 characters long!</h4>";
            credsOk = false;
    }
	if (credsOk == false)
	{
		$("#ap_key_errors").html(errorList);
	}
	else
	{
		$("#ap_key_errors").html("");
		set_ap_key();    
	}
}


function showKey()
{
	var x = document.getElementById("update_key");
	if (x.type === "password")
	{
		x.type = "text";
	}
	else
	{
		x.type = "password";
	}
}

function clearMessages() {
    document.getElementById("ap_key_errors").innerHTML = "";
    document.getElementById("ap_save_status").innerHTML = "";
}







    



