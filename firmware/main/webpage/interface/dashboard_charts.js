document.addEventListener('DOMContentLoaded', function() {
  var waveform1= {}, waveform2= {}, waveform3= {}, waveform4= {}, waveform5= {};
  var newData;
  var conv2imperial = false; 
  let range = [0, 100];
  let labels_data =  [];
  var update_sliders = false;
  range = [0, 100];
  function delayWithCallback(ms, callback, ...args) {
    setTimeout(() => {
        callback(...args);
    }, ms);
  }
  
  let set_slider_on_reload = true;
  const maxRetries = 2; // Maximum number of retry attempts
  let currentRetry = 0; // Current retry attempt

  labels_data = [];
  // Create_Waveform_Chart(waveform3,"waveform-3" ,"Temperature",     range,false,  true, labels_data,0.4);
  // Create_Waveform_Chart(waveform4,"waveform-4" ,"Humidity",       range,false, true, labels_data,0.4);
  
  range = [0, 100];
  Create_Waveform_Chart(waveform5,"waveform-5" ,"Light Density",  range,false, true, labels_data,0.4);
   //var gateway ="ws://localhost:8080"
   var gateway = `ws://${window.location.hostname}/ws`;
   // Create a WebSocket connection
   var socket =  null;
 
  start_websocket_client(50);


    
  var c2i_state = localStorage.getItem("c2i_state");
  if(c2i_state!=null){
    console.log("C2Istate : ",c2i_state);
    conv2imperial = (c2i_state == "true") ? true : false;
    set_m2i(waveform1, waveform2,conv2imperial);
  }
  else {
    conv2imperial = false;
    console.log("cant fetch data from local storage: setting default..");
    set_m2i(waveform1, waveform2,false);
  }
  


  // Create_Waveform_Chart(waveform1,"waveform-1", "Macro Movement", range,true,  true, labels_data , 0.1);
  // Create_Waveform_Chart(waveform2,"waveform-2", "Micro Movement", range,true, true, labels_data , 0.1);
  

  
  // Load and set initial values from localStorage
  loadAndSetInitialValue("max_macro_range", "MR1");
  loadAndSetInitialValue("max_micro_range", "MR2");
  loadAndSetInitialValue("Timeout", "TO");
  // loadAndSetInitialValue("pir_sensitivity", "PS");
  Create_Waveform_Chart(waveform1,"waveform-1", "Macro Movement", range,true,  true, labels_data , 0.1);
  Create_Waveform_Chart(waveform2,"waveform-2", "Micro Movement", range,true, true, labels_data , 0.1);
  
  function set_m2i(waveform1, waveform2, val){
    update_sliders = true;
    if(val){
      
      console.log("Imperial units");
      
      conv2imperial = true;
      labels_data = [0, 2.5, 5, 7.5, 10, 12.5, 15, 17,19.5];  

      changeLabelText("range_label1","0   - 0  ft");
      changeLabelText("range_label2","0   - 2.5ft");
      changeLabelText("range_label3","2.5 - 5  ft");
      changeLabelText("range_label4","5   - 7.5ft");
      changeLabelText("range_label5","7.5 - 10 ft");

      changeLabelText("range_label6","10 - 12.5ft");
      changeLabelText("range_label7","12 - 15  ft");
      changeLabelText("range_label8","15 - 17  ft");
      changeLabelText("range_label9","17 - 19.5ft");  
      changeLabelText("MMR1", "Max Micro Range(ft)");
      changeLabelText("MMR2", "Max Macro Range(ft)");
      changeLabelText("distance_name","Distance (ft)");
    }
    else {
      console.log("Metric units");
      conv2imperial = false;
      labels_data = [0, 75, 150, 225, 300, 375, 450, 525, 600];

      changeLabelText("range_label2","0   - 75  cm");
      changeLabelText("range_label3","75  - 150 cm");
      changeLabelText("range_label4","150 - 225 cm");
      changeLabelText("range_label5","225 - 300 cm");
      changeLabelText("range_label6","300 - 375 cm");
      changeLabelText("range_label7","375 - 450 cm");
      changeLabelText("range_label8","450 - 525 cm");
      changeLabelText("range_label9","525 - 600 cm");
      changeLabelText("range_label1","0 -   0   cm");
      changeLabelText("MMR1", "Max Micro Range(cm)");
      changeLabelText("MMR2", "Max Macro Range(cm)");
      changeLabelText("distance_name","Distance (cm)");
    }
  }
  
  let slider_array1 = [0,0,0,0,0,0,0,0,0] ,slider_array2 = [0,0,0,0,0,0,0,0,0] ;
  // Load and set initial values for Macro and Micro Thresholds
  for (var i = 1; i <= 9; i++) {
    let slider_id1 = "macro_th" + i;
    let slider_id2 = "micro_th" + i;
    console.log("Restoring Slider : %s, %s", slider_id1, slider_id2);
    loadSetInitialSliderValues(slider_id1, i+8, 1);
    loadSetInitialSliderValues(slider_id2, i+8, 2);
  }
  console.log("RESTOREd");
  console.log(slider_array1,slider_array2);
  let lbl = [0, 75, 150, 225, 300, 375, 450, 525, 600];
  update_charts(waveform1.chart,slider_array1, true, labels_data);
  update_charts(waveform2.chart,slider_array2, true, labels_data);
  // Attach change event listener to each slider


  
  var sliders_array = document.querySelectorAll('.slider');
  sliders_array.forEach(function(slider) {
    slider.addEventListener('input', function() {
      var value = this.value;
     // console.log("Saving state!");
      localStorage.setItem(this.id, value);
    });
  });


function loadSetInitialSliderValues(sliderId,i,ctx){
  var savedValue = localStorage.getItem(sliderId);
  console.log("fetched value ",savedValue);
  if (savedValue !== null) {
    document.getElementById(sliderId).value = savedValue;
    if(ctx==1)slider_array1[i] = savedValue;
    else if(ctx ==2)slider_array2[i] = savedValue;
  }
}



function loadAndSetInitialValue(sliderId, labelId) {
  var savedValue = localStorage.getItem(sliderId);
  if (savedValue !== null) {
    document.getElementById(sliderId).value = savedValue;
    document.getElementById(labelId).textContent = savedValue;
  }
}

function update_realtime_values(chart_id, type, value){
  const canvas = document.getElementById(chart_id);
  const context = canvas.getContext('2d');
  context.font = '24px Arial';
  context.fillStyle = 'white';
  let string = "";
  if(type == 'Temperature')  string = value + 'C';
  else if(type == 'Humidity')     string = value + '%';
  else string = value;
  context.fillText(string, 45, 35);
}

let os_last_state = 0xff;
function update_occupation_status(state) {  
	   if(os_last_state[0] !== state[0] )
	   {
	      console.log("Current occ state",state);
	      console.log("Last  state",os_last_state);
	      os_last_state = state;
          var occ_state = document.getElementById("occ_state");
          const statusElement = document.querySelector('.status');
          statusElement.textContent = state;
          if (state == 'Unoccupied') occ_state.src = "home_unoccupid.png";
          else if (state == 'Occupied') occ_state.src = "home_occupid.png";
    }
}

var md_last_state = ["NULL"];

function update_pir_motion_detection_status(state) {
	if(md_last_state[0] != state[0]){
	     console.log("Current motion state",state);
	     console.log("Last motion  state",md_last_state);
         //console.log("Updating motion state :  ", state);
         var motion_status = document.getElementById("motion_status");
         if(state == 'true')motion_status.src = "motion_true.png";
         else if(state == 'false')motion_status.src = "motion_false.png";
         md_last_state = state;
  }
}

function changeLabelText(label_id,text) {
  var label = document.getElementById(label_id);
  label.textContent = text;
}


function start_websocket_client(poll_interval){
    socket = new WebSocket(gateway); // Replace 'wss://example.com' with your WebSocket server URL
    // Handle WebSocket connection opened event
    socket.onopen = function(event) {
      console.log('WebSocket connection opened');
      create_sliders_handlers();        
      create_settings_sliders_handlers();
      create_sliders_blocker_handler();
      add_switches_handlers();
      add_button_handlers();
      setInterval(function() {
      var message = 'data_request';
      socket.send(message);
      }, poll_interval);
      
      console.log("getting config...");
      const data = { "value" : "0" , "name": "get_cnfg" }; 
      socket.send(JSON.stringify(data));  
    };
    
    socket.onclose = function(event) {
    if (currentRetry < maxRetries) {
      currentRetry++;
      console.log(`WebSocket connection closed. Retrying in 2 seconds... (Attempt ${currentRetry}/${maxRetries})`);
      setTimeout(start_websocket_client, 3500); // Retry after 2 seconds
    } else {
      console.error('WebSocket connection failed after maximum retry attempts.');
    }
  };
  
    // Handle WebSocket message received event
    socket.onmessage = function(event) {
      
    //console.log("received ",event.data);  
    newData = JSON.parse(event.data);
    if(1){
      if(newData.settings[0] != "NULL"){
        console.log("received ",event.data);  
        document.getElementById("max_macro_range").value = newData.settings[0];
        document.getElementById("MR1").textContent = (conv2imperial)? Math.round(cm2ft(newData.settings[0])):newData.settings[0];
        document.getElementById("max_micro_range").value = newData.settings[1];
        document.getElementById("MR2").textContent =  (conv2imperial)? Math.round(cm2ft(newData.settings[1])):newData.settings[1];
        document.getElementById("Timeout").value = newData.settings[2];
        document.getElementById("TO").textContent = newData.settings[2];
        // document.getElementById("PS").textContent = newData.settings[3];
        document.getElementById("switch-1").checked = newData.settings[4];
        for(let i = 0; i < 9; i++){
          set_any_slider_range('macro_th', i+1,0,100);
          set_any_slider_by_name('macro_th',i+1,newData.macromovement[i+9]);
        }
        for(let i = 0; i < 9; i++){
            set_any_slider_range('micro_th', i+1,0,100);
            set_any_slider_by_name('micro_th',i+1,newData.micromovement[i+9]);
        }
      }

      if(newData.macromovement[0] != "NULL"){
        if(set_slider_on_reload){
          for(let i = 0; i < 9; i++){
              set_any_slider_by_name('macro_th',i+1,newData.macromovement[i+9]);
          }
        }
          // console.log("Macro:",newData.macromovement);     
        
        update_charts(waveform1.chart,newData.macromovement, true, labels_data);
        
      }
      if(newData.micromovement[0] != "NULL"){ 
        if(set_slider_on_reload){
          set_slider_on_reload = false;
          for(let i = 0; i < 9; i++){
            set_any_slider_by_name('micro_th',i+1,newData.micromovement[i+9]);
          }
        }

        let lbl = [0, 75, 150, 225, 300, 375, 450, 525, 600];
        update_charts(waveform2.chart,newData.micromovement, true, labels_data);

      }
      // //Temperature 
      // if(newData.temperature[0] != "NULL"){
      //   let lbl = [0, 75, 150, 225, 300, 375, 450, 525, 600];
      //   update_chartSP(waveform3.chart,newData.temperature, true,lbl);
      // }

      // if(newData.temperature_realtime[0] != "NULL"){   
      //   update_realtime_values("waveform-3",'Temperature',newData.temperature_realtime[0]);  
      // }

      // //Humidity
      // if(newData.humidity[0] != "NULL"){
      //   let lbl = [0, 75, 150, 225, 300, 375, 450, 525, 600];
      //   update_chartSP(waveform4.chart,newData.humidity, true,lbl);
      // }
      // if(newData.humidity_realtime[0] != "NULL"){
      //   update_realtime_values("waveform-4",'Humidity',newData.humidity_realtime[0]);
      // }

      //Ligh density. 
      if(newData.light_density[0] != "NULL"){
        let lbl = [0, 75, 150, 225, 300, 375, 450, 525, 600];
        update_chartSP(waveform5.chart,newData.light_density, true,lbl);  
      }
      if(newData.light_density_realtime[0] != "NULL"){
        let lbl = [0, 75, 150, 225, 300, 375, 450, 525, 600];
        update_realtime_values("waveform-5",'light_density',newData.light_density_realtime[0]);
      }

      if(newData.distance != "NULL"){
        if(c2i_state == 'true'){
        //console.log(cm2ft(parseInt(newData.distance)));
          document.getElementById('distance_label').textContent = cm2ft(parseInt(newData.distance)).toFixed(1);
        }
        else {
          document.getElementById('distance_label').textContent = newData.distance;
  }
      };


      if(newData.occ_status  != "NULL"){
        update_occupation_status(newData.occ_status);
      }

      if(newData.motion_status != "NULL"){
        update_pir_motion_detection_status(newData.motion_status);
      }
    }
    };
      
  
    // Handle WebSocket connection error event
    socket.onerror = function(error) {
    console.error('WebSocket error:', error);
    };  
}

function cm2ft (cm){
  return (cm * 0.0328);
}

function c2f(c){
  return (c*(9/50)+32);
}

  
function update_chart(ctx, data, append_labels, labels_data) {
  if(append_labels){
    ctx.data.labels = labels_data;
  }
  for (var i = 0; i < data.length; i++) {
    ctx.data.datasets[0].data[i] = data[i];
  }
  // Update the chart
  ctx.update({
    duration: 14,     
    easing: 'linear', 
    preservation: true
  });

}

function update_charts(ctx, dataset, append_labels, labels_data) {
  if(append_labels){
    ctx.data.labels = labels_data;
  }
  for (var i = 0; i < 9; i++) {
    ctx.data.datasets[0].data[i] = dataset[i];
  }

  for (var i = 0; i <9; i++) {
    ctx.data.datasets[1].data[i] = dataset[i+9];
  }

  // Update the chart
  ctx.update({
    duration: 14,     
    easing: 'linear', 
    preservation: true
  });

}

function Create_Waveform_Chart(waveform_obj, elementid, chart_title_name, range, xaxis_display, yaxis_display, labels_data, line_tension) {
  var ctx1 = document.getElementById(elementid).getContext('2d');

  waveform_obj.chart = new Chart(ctx1, {
    type: 'line',
    data: {
      labels: labels_data,
      datasets: [
        {
          data: [],
          borderWidth: 1,
          borderColor: 'orange',
          backgroundColor: 'rgba(0, 0, 0, 0)',
          pointRadius: 0,
          lineTension: line_tension,
          fill: false,
        },
        {
          data: [],
          borderWidth: 1,
          borderColor: '#03a9f4',
          backgroundColor: 'rgba(0, 0, 0, 0)',
          pointRadius: 0,
          lineTension: line_tension,
          fill: false,
        }
      ]
    },
    options: {
      layout: {
        padding: 40
      },
      width: 550,
      height: 350,
      responsive: false,
      backgroundColor: 'rgba(0, 0, 0, 1)',
      scales: {
        x: {
          type: 'category',
          offset: true,
          display: xaxis_display,
        },
        x2: {
          type: 'category',
          offset: true,
          display: false, // Set display to false to hide the x2-axis labels
        },
        y: {
          display: yaxis_display,
          grid: {
            color: 'rgba(255, 255, 255, 0.1)'
          },
          suggestedMin: range[0],
          suggestedMax: range[1]
        },

      },
      elements: {
        line: {
          backgroundColor: 'orange',
          borderColor: 'orange',
          tension: 0.3
        }
      },
      animation: {
        duration: 0
      },
      interaction: {
        mode: 'nearest',
        intersect: false
      },
      plugins: {
        title: {
          color: 'rgba(255, 255, 255, 1)',
          text: chart_title_name,
          display: true
        },
        legend: {
          display: false
        },
        streaming: {
          frameRate: 30,
          duration: 10000,
          delay: 0
        }
      }
    }
  });
}
function delay(ms) {
  return new Promise(resolve => setTimeout(resolve, ms));
}

function update_chartSP(ctx, newData, appendLabels, labelsData) {
  const maxPoints = 96;

  if (appendLabels) {
    // Append new labels to the existing labels
    ctx.data.labels.push(...labelsData);
    // Ensure there are no more than 16 labels
    if (ctx.data.labels.length > maxPoints) {
      ctx.data.labels = ctx.data.labels.slice(-maxPoints);
    }
  }

  // Append new data points to the existing dataset
  ctx.data.datasets[0].data.push(...newData);
  // Ensure there are no more than 16 data points
  if (ctx.data.datasets[0].data.length > maxPoints) {
    ctx.data.datasets[0].data = ctx.data.datasets[0].data.slice(-maxPoints);
  }

  // Update the chart
  ctx.update({
    duration: 14,
    easing: 'linear',
    preservation: true
  });
}

function set_slider_by_name (slider_index,slider_value, slider_state){
  slider_index =  parseInt(slider_index);
  slider_index = slider_index + 1; 
  slider_index = slider_index.toString(); 
  let macroid = 'macro_th'+slider_index ;
  let microid = 'micro_th'+slider_index ;
  // console.log("Setting slider %d |  %s", slider_index, macroid);
  let macro_slider = document.getElementById(macroid);
  let micro_slider = document.getElementById(microid); 
  macro_slider.value = slider_value;
  micro_slider.value = slider_value;

  macro_slider.disabled = slider_state;
  micro_slider.disabled = slider_state;
  send_json_over_ws(macro_slider);
  send_json_over_ws(micro_slider);
}



function create_sliders_blocker_handler(){
  for(let i = 0; i < 9; i++){
    let blocker_id = 'slider_blocker'+i;
    let blocker_cb = document.getElementById(blocker_id);
    blocker_cb.addEventListener('change', function(event) {
      let name = event.target.id;
      set_slider_by_name(name[14], blocker_cb.checked? 100 : 0,blocker_cb.checked);
    });
  }     
}

function create_sliders_handlers(){
    for(let i = 1; i < 10; i++){
      let macroid = 'macro_th'+i;
      let microid = 'micro_th'+i;
      let macro_slider = document.getElementById(macroid);
      let micro_slider = document.getElementById(microid); 
      
      macro_slider.addEventListener('input', function() {
        send_json_over_ws(macro_slider);
      });

      micro_slider.addEventListener('input', function() {
        send_json_over_ws(micro_slider);
      });
    }     
}

function create_settings_sliders_handlers(){
  let max_macro_range = document.getElementById("max_macro_range"); 
  max_macro_range.addEventListener('input', function() {      
    var label = document.getElementById('MR1');
    label.textContent = (conv2imperial)? Math.round(cm2ft(max_macro_range.value)):max_macro_range.value;
    send_json_over_ws(max_macro_range);
  });

  let max_micro_range = document.getElementById("max_micro_range");        
  max_micro_range.addEventListener('input', function() {
    var label = document.getElementById('MR2');
    label.textContent = (conv2imperial)? Math.round(cm2ft(max_micro_range.value)):max_micro_range.value;            
    send_json_over_ws(max_micro_range);
  });

  let Timeout = document.getElementById("Timeout");    
         
    Timeout.addEventListener('input', function() {      
      var label = document.getElementById('TO');
      label.textContent =Timeout.value; 
      send_json_over_ws(Timeout);
    });
}

function send_json_over_ws(slider){
  let value = slider.value;
  let name = slider.id; 
  const data = {value, name }; 
  //console.log('Slider Context:', data);
  socket.send(JSON.stringify(data));
}

function send_ws_msg(msg){
  socket.send(JSON.stringify(msg));
}



function add_switches_handlers(){
  

  var sw1 = document.getElementById('switch-1'); 
  sw1.addEventListener('change', function() {
    let name = "BedSense";
    let value = sw1.checked? "1" : "0";
    const SWdata = {name, value};
    //console.log(JSON.stringify( SWdata));
    socket.send(JSON.stringify( SWdata));
  });


}



function set_any_slider_range(slider_prefix, slider_index,min,max){
  let sliderid = slider_prefix+slider_index ;
  let sliderObj = document.getElementById(sliderid);
  if (sliderObj != null){
    sliderObj.min = min;
    sliderObj.max = max;
  }
  else console.log("ERROR AT : ", sliderid);
}


function set_any_slider_by_name(slider_prefix, slider_index,slider_value){
  let sliderid = slider_prefix+slider_index ;
  let sliderObj = document.getElementById(sliderid);
  if (sliderObj != null)sliderObj.value = slider_value;
  else console.log("ERROR AT : ", sliderid);
}

function add_button_handlers(){
  function handleGetConfig() {
    console.log("getting config...");
    const data = { "value" : "0" , "name": "get_cnfg" }; 
    console.log('Slider Context:', JSON.stringify(data));
    
    
    socket.send(JSON.stringify(data));   
  }

  function handleFactoryReset() {
    console.log("Factory Reset button clicked!");
    const data = { "value" : "0" , "name": "factory_reset" }; 
    console.log('Slider Context:', JSON.stringify(data));
    socket.send(JSON.stringify(data));
  }
  

  function handleAutoCalibration() {
    document.getElementById('bcal-btn').textContent = "Calibrating...";
    const data = { "value" : "0" , "name": "baseline_calibration" }; 
    console.log('Slider Context:', JSON.stringify(data));
    socket.send(JSON.stringify(data));
    delayWithCallback(10000, () => {
        console.log("Done calibration...");
        document.getElementById('bcal-btn').textContent = "Baseline Calibration";
    });
  }


  document.getElementById("bscal-btn").addEventListener('click',function(){
    document.getElementById('bscal-btn').textContent = "Calibrating...";
    const data = { "value" : "0" , "name": "blind_spot_calibration" }; 
    console.log('Slider Context:', JSON.stringify(data));
    socket.send(JSON.stringify(data));
    delayWithCallback(10000, () => {
        console.log("Done blindspot calibration...");
        document.getElementById('bscal-btn').textContent = "Blind spot Calibration";
    });
  });
  

  // Get references to the buttons
  const getConfigButton = document.getElementById('get_cnfg');
  const factoryResetButton = document.getElementById('f_reset');
  const autoCalibrationButton = document.getElementById('bcal-btn');
  
  // Add event listeners to the buttons
  getConfigButton.addEventListener('click', handleGetConfig);
  factoryResetButton.addEventListener('click', handleFactoryReset);
  autoCalibrationButton.addEventListener('click', handleAutoCalibration);
}

});



