#!/usr/bin/env python3

## @package network_manager
#  Allows communication with the gateway to monitor and control the network
#
#  This is a python application with a gui designed to interface with the wireless sensor network through
#  the serial port connected to the gateway. It allows for downlink messages to be sent, uplink messages to
#  be received and monitoring of the network. Additionally, network tests can be run and the data monitored
#  can be exported for further analysis

import rospy
from std_msgs.msg import String
pub = rospy.Publisher('/gateway/ul', String, queue_size=10)

import serial
from parse import parse
import threading
import PySimpleGUI as sg
import json 
from datetime import datetime
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
plt.rcParams.update({'font.size': 6})
import yaml
import csv
import time


# Global variables declaration.
maxNum = 5
gateway_status = "Offline"

_VARS = {'rssi_canvas': None,
         'snr_canvas': None,
		 'dl_msgs': {
			'timestamps': list(),
			'nodeID': list(),
			'msgID': list(),
			'delay': list(),
		 }}


## Function that runs a test on the network
def network_test():
	runs = 5
	msg_delay = 3
	print('Starting test!')
	for i in range(runs):
		global stop_threads
		for node in nodes:
			if stop_threads:
				return
			data = 's,' + str(node['id'])
			send_dl_msg('s,2')
			#print(datetime.now(), data)
			time.sleep(msg_delay)
	print('Test finished!')

def idxFromID(id):
	idx = -1

	for i in range(len(nodes)):
		if nodes[i]['id'] == id:
			idx = i
			break		
	return idx
	

## Function that draws a plot onto a figure
def draw_figure(canvas, figure):
	figure_canvas_agg = FigureCanvasTkAgg(figure, canvas)
	figure_canvas_agg.draw_idle()
	figure_canvas_agg.get_tk_widget().pack(side='top', fill='both', expand=1)
	return figure_canvas_agg

## Function that sends a downlink message to the gateway through the serial connection
def send_dl_msg(data):
	data = bytes(data, encoding='utf-8')
	ser.write(data)
	ser.write(bytes("\n", encoding='utf-8'))
	ser.flush()

## Function to export the gathered data onto a .csv file
def export_data(path):
	#print(path)
	with open(path, 'w', newline='') as csvfile:
		writer = csv.writer(csvfile, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
		for node in nodes:
			for i in range(len(node['timestamps'])):
				writer.writerow([node['timestamps'][i], node['delay_list'][i], node['msgID_list'][i], node['id'], node['rssi_list'][i], node['snr_list'][i], node['battery_list'][i], '0'])
		for i in range(len(_VARS['dl_msgs']['timestamps'])):
			writer.writerow([_VARS['dl_msgs']['timestamps'][i], _VARS['dl_msgs']['delay'][i], _VARS['dl_msgs']['msgID'][i], _VARS['dl_msgs']['nodeID'][i], 0, 0, 0, '1'])
	print('Data exported!')
	#timestamp, nodeID, rssi, snr, battery, DL/UL
	#if DL we only care about timestamp and nodeID
	# set up custom commands for DLMSG: test1, test2 -> execute predetermined tests and save data

## Function responsible for all the GUI initialization and layout
def gui():
	sg.theme('BrownBlue') 
	global active_nodes
	global nodes
	global total_nodes

	node_keys = {'Node ' + str(node['id']): node for node in nodes}

	sensor_pane_layout = [ sg.Column([
		[sg.Text('Sensor ID:', background_color='white', text_color='black'), sg.Text('id', background_color='white', text_color='black', key='_SID'+str(i)+'_')],
		[sg.Text('Sensor Name:', background_color='white', text_color='black'), sg.Text('name', background_color='white', text_color='black', key='_SNAME'+str(i)+'_')],
		[sg.Text('Last Activity:', background_color='white', text_color='black'), sg.Text('never', background_color='white', text_color='black', key='_SLA'+str(i)+'_')],
		[sg.Text('State:', background_color='white', text_color='black'), sg.Text('unknown', background_color='white', text_color='black', key='_SSTATE'+str(i)+'_')],
						], expand_y = True, background_color='white', key='_SCOL'+str(i)+'_', visible=False) for i in range(maxNum)]

	actuator_pane_layout = [ sg.Column([
		[sg.Text('Actuator ID:', background_color='white', text_color='black'), sg.Text('id', background_color='white', text_color='black', key='_AID'+str(i)+'_')],
		[sg.Text('Actuator Name:', background_color='white', text_color='black'), sg.Text('name', background_color='white', text_color='black', key='_ANAME'+str(i)+'_')],
		[sg.Text('Last Activity:', background_color='white', text_color='black'), sg.Text('never', background_color='white', text_color='black', key='_ALA'+str(i)+'_')],
		[sg.Text('State:', background_color='white', text_color='black'), sg.Text('unknown', background_color='white', text_color='black', key='_ASTATE'+str(i)+'_')],
		[sg.Button('ON', key='_AON'+str(i)+'_'),sg.Button('OFF', key='_AOFF'+str(i)+'_')],
						], expand_y = True, background_color='white', key='_ACOL'+str(i)+'_', visible=False) for i in range(maxNum)]

	status_tab_layout = [
		[
			sg.Text('Node State:', background_color='white', text_color='black'),
			sg.Text('Offline', key='_NSTATE_', background_color='white', text_color='red')
		],
		[
			sg.Text('Node Location:', background_color='white', text_color='black'),
			sg.Text('', key='_NLOCATION_', background_color='white', text_color='black')
		],
		[
			sg.Text('Last Activity:', background_color='white', text_color='black'),
			sg.Text('None', key='_NLASTACTIVITY_', background_color='white', text_color='black')
		],
		[
			sg.Text('Sensors:', background_color='white', text_color='black'),
		],
		[
			sg.Column( layout=[sensor_pane_layout], key='_SENSORSPANE_', scrollable = True, size = (None, 110), expand_x=True)
		],
		[
			sg.Text('Actuators:', background_color='white', text_color='black'),
		],
		[
			sg.Column( layout=[actuator_pane_layout], key='_ACTUATORSPANE_', scrollable = True, size = (None, 145), expand_x=True)
		]
	]
	info_tab_layout = [
		[
			sg.Text('Packets sent:', background_color='white', text_color='black'),
			sg.Text('0', key='_PACKETSSENT_', background_color='white', text_color='black')
		],
		#[
		#	sg.Text('Packets received:', background_color='white', text_color='black'),
		#	sg.Text('0', key='_PACKETSRECEIVED_', background_color='white', text_color='black')
		#],
		[
			sg.Text('Average RSSI:', background_color='white', text_color='black'),
			sg.Text('', key='_AVGRSSI_', background_color='white', text_color='black')
		],
		[
			sg.Text('Average SNR:', background_color='white', text_color='black'),
			sg.Text('', key='_AVGSNR_', background_color='white', text_color='black')
		],
		[
			sg.Text('Battery Level (V):', background_color='white', text_color='black'),
			sg.Text('', key='_BAT_', background_color='white', text_color='black')
		],
		[
			sg.Column(layout = [
				[
					sg.Frame(layout = [[sg.Canvas(key='rssi_canvas', background_color=sg.theme_background_color())]], title = "RSSI vs Packet", size=(250,250)),
					sg.Frame(layout = [[sg.Canvas(key='snr_canvas', background_color=sg.theme_background_color())]], title = "SNR vs Packet", size=(250,250))
				]
			], scrollable=False, expand_x = True, expand_y = True)
		]
	]

	layout = [  [
					sg.Text('Active Nodes:'),
					sg.Text(str(active_nodes), key='_ACTIVENODES_'),
					sg.Text('/'),
					sg.Text( str(total_nodes), key='_TOTALNODES_')
				],
				[
					sg.Text('Gateway Status:'),
					sg.Text(gateway_status, key='_GATEWAYSTATUS_', text_color='red'),
					sg.Push(),
					sg.Text('Boot time:'),
					sg.Text('', key='_BOOTTIME_')
				],
				[sg.Text('Send Downlink Message'), sg.InputText(key='_DLMSG_'), sg.Button('Send')],
				[sg.HorizontalSeparator()],
				[
					sg.Listbox(values=list(node_keys.keys()), size=(15, 27), key='_LIST_', enable_events=True, select_mode='LISTBOX_SELECT_MODE_SINGLE', default_values=['Node 1']),
					sg.VerticalSeparator(),
					sg.TabGroup([
						[sg.Tab(title='Node 1 Status', key='_STATUSTAB_', background_color='white', layout = status_tab_layout)],
						[sg.Tab(title='Node 1 Stats', key='_STATSTAB_', background_color='white', layout = info_tab_layout)]
					], 	tab_location='topleft', 
						size=(600,None),
						selected_background_color='white', 
						tab_background_color=sg.theme_background_color(), 
						selected_title_color=sg.theme_background_color(), 
						title_color='white'					
					)
				],
				[sg.Output(size=(119,8))],
				[
					sg.Button('Rescan Network', key='_RSNET_'), 
					sg.Button('Start Test', key='_TEST_'), 
					sg.InputText(visible=False, enable_events=True, key='export_data_path'),
					sg.FileSaveAs('Export data', key='_EXPORT_', file_types=(('CSV', '.csv'),)),
					sg.Push(),
					sg.Button('Exit')
				] 
			]

	global window
	window = sg.Window('Sensor Network Manager', layout, finalize=True)
	
	for i in range(len(nodes)):
		plt.figure(num=0)
		plt.plot([],[],'.k')
		#nodes[i]['rssi_plot_canvas'] = FigureCanvasTkAgg(plt.figure(num=0), window.Element('rssi_canvas').TKCanvas)
		_VARS['rssi_canvas'] = FigureCanvasTkAgg(plt.figure(num=0), window.Element('rssi_canvas').TKCanvas)
		#nodes[i]['rssi_plot_canvas'].get_tk_widget().hide()
		nodes[i]['rssi_list'] = list()
		plt.figure(num=1)
		plt.plot([],[],'.k')
		#nodes[i]['snr_plot_canvas'] = FigureCanvasTkAgg(plt.figure(num=1), window.Element('snr_canvas').TKCanvas)
		_VARS['snr_canvas'] = FigureCanvasTkAgg(plt.figure(num=1), window.Element('snr_canvas').TKCanvas)
		nodes[i]['snr_list'] = list()

		nodes[i]['timestamps'] = list()
		nodes[i]['battery_list'] = list()
		nodes[i]['msgID_list'] = list()
		nodes[i]['delay_list'] = list()

	while True:
		event, values = window.read(timeout = 200)
		if event == sg.WIN_CLOSED or event == 'Exit': 
			break
		if event == '_TEST_':
			global nt_thread
			nt_thread.start()
		if event == 'export_data_path':
			path = values['export_data_path']
			export_data(path)
		if event == 'Send' and len(values['_DLMSG_']):
			data = values['_DLMSG_']
			send_dl_msg(data)
			window.Element('_DLMSG_').update(value="")
    		
		if event == '_LIST_' and len(values['_LIST_']): 
			r = parse("[\'Node {}\']", str(values['_LIST_']))
			idx = int(r[0])
			idx = idxFromID(idx)
			window.Element('_STATUSTAB_').update(title='Node ' + str(nodes[idx]['id']) + ' Status')
			window.Element('_STATSTAB_').update(title='Node ' + str(nodes[idx]['id']) + ' Stats')
			updateTabs(idx)
		if event == '_RSNET_':
			active_nodes = 0
			window.Element('_ACTIVENODES_').update(value=str(active_nodes))
			for i in range(len(nodes)):
				nodes[i]["state"] = 0 
			r = parse("[\'Node {}\']", str(values['_LIST_']))
			idx = int(r[0])-1
			#updateTabs(idx)
			send_dl_msg("s,-1\n")
		if '_AON' in event:
			r = parse("[\'Node {}\']", str(values['_LIST_']))
			nID = int(r[0])
			r = parse("_AON{}_", str(event))
			actID = int(r[0])
			actID = nodes[nID-1]['actuators'][actID]['id']

			data = 'c,' + str(nID) + ',' + str(actID) + ',1'
			send_dl_msg(data)

		if '_AOFF' in event:
			r = parse("[\'Node {}\']", str(values['_LIST_']))
			nID = int(r[0])
			r = parse("_AOFF{}_", str(event))
			actID = int(r[0])
			actID = nodes[nID-1]['actuators'][actID]['id']

			data = 'c,' + str(nID) + ',' + str(actID) + ',0'
			send_dl_msg(data)
		
		window.refresh()

	window.close()

firstTime = True

## Function that updates the GUI
def updateTabs(idx):
	global nodes
	global window

	window.Element('_NSTATE_').update(value='Online' if int(nodes[idx]['state']) else 'Offline', text_color='green' if int(nodes[idx]['state']) else 'red')
	window.Element('_NLASTACTIVITY_').update(value=str(nodes[idx]['last_activity']))
	window.Element('_NLOCATION_').update(value=str(nodes[idx]['location']))

	window.Element('_PACKETSSENT_').update(value=str(nodes[idx]['packets_sent']))
	#window.Element('_PACKETSRECEIVED_').update(value=str(nodes[idx]['packets_received']))
	window.Element('_PACKETSSENT_').update(value=str(nodes[idx]['packets_sent']))
	window.Element('_AVGRSSI_').update(value=str(nodes[idx]['avg_rssi']))
	window.Element('_AVGSNR_').update(value=str(nodes[idx]['avg_snr']))
	window.Element('_BAT_').update(value=str(nodes[idx]['battery']))

	# \\  -------- PYPLOT -------- //
	if(len(nodes[idx]['rssi_list'])>0):
		#nodes[idx]['rssi_plot_canvas'].get_tk_widget().forget()
		_VARS['rssi_canvas'].get_tk_widget().forget()
		plt.figure(num=0)
		plt.clf()
		plt.plot(range(1,len(nodes[idx]['rssi_list'])+1), nodes[idx]['rssi_list'], 'bo-', linewidth=0.5, markersize=3)
		plt.xticks(np.arange(1, len(nodes[idx]['rssi_list'])+1))
		plt.ylim(-120, 20)
		#nodes[idx]['rssi_plot_canvas'] = draw_figure(window.Element('rssi_canvas').TKCanvas, plt.figure(num=0))
		_VARS['rssi_canvas'] = draw_figure(window.Element('rssi_canvas').TKCanvas, plt.figure(num=0))

		#nodes[idx]['snr_plot_canvas'].get_tk_widget().forget()
		_VARS['snr_canvas'].get_tk_widget().forget()
		plt.figure(num=1)
		plt.clf()
		plt.plot(range(1,len(nodes[idx]['snr_list'])+1), nodes[idx]['snr_list'], 'bo-', linewidth=0.5, markersize=3)
		plt.xticks(np.arange(1, len(nodes[idx]['snr_list'])+1))
		plt.ylim(-15, 15)
		#nodes[idx]['snr_plot_canvas'] = draw_figure(window.Element('snr_canvas').TKCanvas, plt.figure(num=1))
		_VARS['snr_canvas'] = draw_figure(window.Element('snr_canvas').TKCanvas, plt.figure(num=1))
	# \\  -------- PYPLOT -------- //

	for i in range(maxNum):
		if i < len(nodes[idx]['sensors']):
			window.Element('_SCOL'+str(i)+'_').update(visible=True)
			window.Element('_SID'+str(i)+'_').update(value=str(nodes[idx]['sensors'][i]['id']))
			window.Element('_SNAME'+str(i)+'_').update(value=str(nodes[idx]['sensors'][i]['name']))
			window.Element('_SLA'+str(i)+'_').update(value=str(nodes[idx]['sensors'][i]['last_activity']))
			window.Element('_SSTATE'+str(i)+'_').update(value=str(nodes[idx]['sensors'][i]['state']))
		else:
			window.Element('_SCOL'+str(i)+'_').update(visible=False)
	

	for i in range(maxNum):
		if i < len(nodes[idx]['actuators']):
			window.Element('_ACOL'+str(i)+'_').update(visible=True)
			window.Element('_AID'+str(i)+'_').update(value=str(nodes[idx]['actuators'][i]['id']))
			window.Element('_ANAME'+str(i)+'_').update(value=str(nodes[idx]['actuators'][i]['name']))
			window.Element('_ALA'+str(i)+'_').update(value=str(nodes[idx]['actuators'][i]['last_activity']))
			window.Element('_ASTATE'+str(i)+'_').update(value=str(nodes[idx]['actuators'][i]['state']))
		else:
			window.Element('_ACOL'+str(i)+'_').update(visible=False)

	window.Element('_SENSORSPANE_').contents_changed()
	window.Element('_ACTUATORSPANE_').contents_changed()


def gateway_dl_callback(data):
	r = parse("data: \"{}\"", str(data))
	send_dl_msg(r[0])

## Function that handles the received messages from the gateway through the serial communication
def serial_comm():
	while  True:
		global stop_threads
		if stop_threads:
			break
		global nodes
		global window

		line = ser.readline()
		line = line.decode('utf-8', "ignore")
		if(len(line) > 4):
			line = line.strip('rm').strip('\n').rstrip()
			if line == "Startup complete":
				time.sleep(5)
				window.Element('_GATEWAYSTATUS_').update(value=line, text_color='#42cf68')
				window.Element('_BOOTTIME_').update(value=datetime.now().strftime("%d/%m/%Y %H:%M:%S"))
			else:
				try:
					print(datetime.now(), line)
					msg = json.loads(line)

					if(msg['f'] == 'd'):
						_VARS['dl_msgs']['nodeID'].append(msg['nID'])
						_VARS['dl_msgs']['timestamps'].append(datetime.now())
						_VARS['dl_msgs']['msgID'].append(msg['msgID'])
						_VARS['dl_msgs']['delay'].append(msg['t'])
					else:
						pub.publish(String(line))

						nidx = idxFromID(int(msg['nID']))
						now = datetime.now()
						dt_string = now.strftime("%d/%m/%Y %H:%M:%S")

						if(int(msg['RSSI']) != 0):
							nodes[idxFromID(int(msg['nID']))]['packets_sent'] += 1	
							t_packets = nodes[idxFromID(int(msg['nID']))]['packets_sent'] + nodes[idxFromID(int(msg['nID']))]['packets_received']
							avg_rssi = float(nodes[idxFromID(int(msg['nID']))]['avg_rssi']) * float(t_packets-1)/t_packets + float(msg['RSSI']) * float(1/t_packets)
							avg_snr = nodes[idxFromID(int(msg['nID']))]['avg_snr'] * float(t_packets-1)/t_packets + float(msg['SNR']) * float(1/t_packets)
							bat = float(msg['VBAT'])
							
							nodes[idxFromID(int(msg['nID']))]['avg_rssi'] = round(avg_rssi, 2)
							nodes[idxFromID(int(msg['nID']))]['avg_snr'] = round(avg_snr, 2)
							nodes[idxFromID(int(msg['nID']))]['battery'] = round(bat, 1)

							for node in nodes:
								if int(node['id']) == int(msg['nID']):
									node['rssi_list'] += [float(msg['RSSI'])]
									node['snr_list'] += [float(msg['SNR'])]
									node['battery_list'] += [float(msg['VBAT'])]
									node['timestamps'] += [datetime.now()]
									node['msgID_list'] += [int(msg['msgID'])]
									node['delay_list'] += [msg['t']]

						if((int(msg['nID']) != 255) and (msg['f'] == 's')):
							nodes[idxFromID(int(msg['nID']))]['state'] = int(msg['state'])

							active_nodes = sum(node["state"] == 1 for node in nodes)
							window.Element('_ACTIVENODES_').update(value=str(active_nodes))
							nodes[nidx]['last_activity'] = 'state update' + ' at ' + dt_string
							if(nidx == window.Element('_LIST_').get_indexes()[0]):
								updateTabs(nidx)

						if((int(msg['nID']) != 255) and (msg['f'] == 'u')):
							nodes[nidx]['last_activity'] = str(nodes[nidx]['sensors'][int(msg['sID'])-1]['name']) + ' with value: ' + msg['sVal'] + ' at ' + dt_string
							window.Element('_LIST_').update(set_to_index=nidx)
							window.Element('_STATUSTAB_').update(title='Node ' + msg['nID'] + ' Status')
							window.Element('_STATSTAB_').update(title='Node ' + msg['nID'] + ' Info')

							
							nodes[nidx]['sensors'][int(msg['sID'])-1]['last_activity'] = dt_string
							nodes[nidx]['sensors'][int(msg['sID'])-1]['state'] = msg['sVal']

							nodes[idxFromID(int(msg['nID']))]['state'] = 1
							active_nodes = sum(node["state"] == 1 for node in nodes)
							window.Element('_ACTIVENODES_').update(value=str(active_nodes))
							updateTabs(nidx)
						if((int(msg['nID']) != 255) and (msg['f'] == 'a')):
							nodes[nidx]['last_activity'] = str(nodes[nidx]['actuators'][int(msg['actID'])-1]['name']) + ' with value: ' + msg['actVal'] + ' at ' + dt_string
							window.Element('_LIST_').update(set_to_index=nidx)
							window.Element('_STATUSTAB_').update(title='Node ' + msg['nID'] + ' Status')
							window.Element('_STATSTAB_').update(title='Node ' + msg['nID'] + ' Info')
							nodes[nidx]['actuators'][int(msg['actID'])-1]['last_activity'] = dt_string
							nodes[nidx]['actuators'][int(msg['actID'])-1]['state'] = msg['actVal']
							nodes[idxFromID(int(msg['nID']))]['state'] = 1
							active_nodes = sum(node["state"] == 1 for node in nodes)
							window.Element('_ACTIVENODES_').update(value=str(active_nodes))
							if(nidx == window.Element('_LIST_').get_indexes()[0]):
								updateTabs(nidx)
					
				except:
					continue
					#print("ERROR reading from serial!!")
def gateway_serial():
	rate = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():
		rate.sleep()


sc_thread = threading.Thread(target=serial_comm)
nt_thread = threading.Thread(target=network_test)
gs_thread = threading.Thread(target=gateway_serial)


## Main function
def main():
	global active_nodes
	active_nodes = 0
	global total_nodes
	global ser
	global nodes
	global stop_threads
	stop_threads = False

	rospy.init_node('gateway_serial')
	rospy.Subscriber("/gateway/dl", String, gateway_dl_callback)

	''' with open("../config/wsn_config.yaml", "r") as stream:
		try:
			config = yaml.safe_load(stream)
		except yaml.YAMLError as exc:
			print(exc)

	gateways = config['wsn_config']['gateways']
	for i in range(len(gateways)):
		gateways_data = {
			'state': 0
		}
		gateways[i] = {**gateways[i], **gateways_data}

	nodes = config['wsn_config']['nodes'] '''

	gateways = rospy.get_param("/wsn_config/gateways")
	for i in range(len(gateways)):
		gateways_data = {
			'state': 0
		}
		gateways[i] = {**gateways[i], **gateways_data}

	nodes = rospy.get_param('/wsn_config/nodes')

	node_data = {
		'state': 0,
		'last_activity': '',
		'packets_sent':0,
		'packets_received':0,
		'avg_rssi': 0,
		'avg_snr': 0,
		'battery': 0,
		'timestamps': list(),
		'rssi_list': list(),
		'snr_list': list(),
		'battery_list': list(),
		'msgID_list': list(),
		'delay_list': list(),
		'rssi_plot_canvas': None,
		'snr_plot_canvas': None
	}

	sensors_data = {
		'last_activity': '',
		'state': None
	}
	actuators_data = {
		'last_activity': '',
		'state': None
	}
	for i in range(len(nodes)):
		for j in range(len(nodes[i]['sensors'])):
			nodes[i]['sensors'][j] = {**nodes[i]['sensors'][j], **sensors_data}
		for j in range(len(nodes[i]['actuators'])):
			nodes[i]['actuators'][j] = {**nodes[i]['actuators'][j], **actuators_data}
		nodes[i] = {**nodes[i], **node_data}

	total_nodes = len(nodes)

	try:
		ser = serial.Serial(gateways[0]['serial_port'], 9600, timeout=1)
	except:
		print("Serial port not available!")
		return

	sc_thread.start()

	
	gs_thread.start()

	gui()

	stop_threads = True
	sc_thread.join()
	#nt_thread.join()

	ser.close()

	return 0

if __name__ == "__main__":
    main()



	
	


