

import os, sys, math

#sys.path.append('~/Dropbox/Qontrol/Code/Application/')

import qontrol
from keithley2000 import Keithley2000
from keithley2231A import Keithley2231A
import pylab, numpy, copy, re, time, glob, random, datetime, os

normal_text = "\033[0m"
emph_text = "\033[97;1m"
good_text = "\033[92;1m"
bad_text = "\033[31;1;5m"
warn_text = "\033[93;1m"

class Tee(object):
	"""Copy stdout to a file"""
	def __init__(self, name, mode):
		self.file = open(os.path.expanduser(name), mode)
		self.stdout = sys.stdout
		sys.stdout = self
	def flush(self):
		self.file.flush()
		self.stdout.flush()
	def __del__(self):
		sys.stdout = self.stdout
		self.file.close()
	def write(self, data):
		self.stdout.write(data)
		try:
			data = data.replace(normal_text, "")
			data = data.replace(emph_text, "")
			data = data.replace(good_text, "")
			data = data.replace(bad_text, "")
			data = data.replace(warn_text, "")
		except:
			pass
		self.file.write(data)
	def write_to_file_only(self, data):
		self.file.write(data)
	def write_to_screen_only(self, data):
		self.stdout.write(data)


from subprocess import call

def nap(time):
	"""Hacked sleep function to use system sleep, not Python one (which falls down in priority sometimes and causes super slowness) JWS Jan 2019"""
	if time:
		call(["sleep",str(time)])
	return


def setup_qontroller(serial_port_name = None, timeout = 0.050, n_chs = 8, catch_errs = True):
	
	# Find serial port
	if serial_port_name is None:
		serial_ports = glob.glob('/dev/tty.usbserial-FT*')
		if len(serial_ports) > 1:
			raise RuntimeError('Found multiple potential serial ports. Specify the correct one manually. Ports are {:}'.format(serial_ports))
	
		serial_port_name = serial_ports[0]
		#print ("Using serial port '{:}'.".format(serial_port_name))

	# Setup Qontroller
	Q = qontrol.Qontroller(serial_port_name = serial_port_name)
	
	dtype = Q.chain[0]["device_type"]
	
	del(Q)
	
	if dtype in {'Q8iv','Q8b'}:
		q = qontrol.QXOutput(serial_port_name = serial_port_name, response_timeout = timeout, n_chs = n_chs)
	elif dtype in {'M2'}:
		q = qontrol.MXMotor(serial_port_name = serial_port_name, response_timeout = timeout, n_chs = n_chs)
	else:
		raise RuntimeError("Cannot setup unrecognised device of type '{:}'".format(dtype))

	# Setup function to handle log messages as they come
	if catch_errs:
		def my_log_handler(err_dict):
			try:
				if err_dict['type'] == 'err':
					print ('Caught Qontrol error "{1}" ({b}{2}{n}) at {0:.1f} ms'.format(1000*err_dict['proctime'], err_dict['desc'], err_dict['raw'].rstrip(), b=bad_text, n=normal_text))
					if err_dict['id'] in [15,10,11,12,0]:
						print (' Last command was {:}'.format(q.log[-2]['desc']))
			except:
				print ('Caught error in error handler... FML')

		q.log_handler = my_log_handler
	
	# Return initialised qontroller
	return q




def program_device(flash_script_filename, device_type):
	print ('\nProgramming device...')
	import subprocess
	result = subprocess.call([os.path.expanduser(flash_script_filename), device_type])
	print ('  Result:  {:}'.format(result) )
	nap(1)
	return result


def get_XY(
		qontroller, 
		keithley,
		xmin = 0.0,
		xmax = 12.0, 
		xstep = 0.1, 
		set_variable = 'v',
		measured_variables = ['v'],
		measured_slope_tolerance_ranges = [],
		measured_offset_tolerance_ranges = [],
		filename = '', 
		plot = True,
		plot_deviation = True,
		rawi = True,
		rawv = True,
		ignore_limits = True,
		random_order = False,
		print_uncertainties = True,
		failure_repeats = 2,
		dwell_t = 0,
		channels = range(8),
		scan_keithley = False):
	
	"""
	Sweep X, measure Y(s).
	
	measured_slope_tolerance_ranges:	(min,max) tolerance slope for colouring. 
										Length to match measured_variables.
										Default [] means 'do not colour', same as ()
											e.g. [(0,2), (), ()] means check first Y, not others
	measured_offset_tolerance_ranges:	Same as above, but for offsets (b)
	"""
	
	# Keep track of success
	qcpass = True
	
	# Normalise case
	set_variable = set_variable.upper()
	for i in range(len(measured_variables)):
		measured_variables[i] = measured_variables[i].upper()
		
	# Check tolerance check specs
	if len(measured_slope_tolerance_ranges) == 0:
		measured_slope_tolerance_ranges = [() for _ in range(len(measured_variables))]
	elif len(measured_slope_tolerance_ranges) != len(measured_variables):
		raise AttributeError("measured_slope_tolerance_ranges has wrong number of elements ({:}); should be {:}, to match requested number of variables.".format(len(measured_slope_tolerance_ranges), len(measured_variables)))
		
		
	if len(measured_offset_tolerance_ranges) == 0:
		measured_offset_tolerance_ranges = [() for _ in range(len(measured_variables))]
	elif len(measured_offset_tolerance_ranges) != len(measured_variables):
		raise AttributeError("measured_offset_tolerance_ranges has wrong number of elements ({:}); should be {:}, to match requested number of variables.".format(len(measured_offset_tolerance_ranges), len(measured_variables)))
		
	# Get metadata
	if set_variable == 'V':
		set_label = 'voltage'
		set_units = 'V'
	if set_variable == 'I':
		set_label = 'current'
		set_units = 'mA'
	
	print ('\nCharacterising {:}{x}-{y} curves...'.format('uncalibrated ' if rawi or rawv else '', x=set_variable, y=measured_variables))
	print ('  Measured quantities: V = external; U = on-board; I = on-board')
	
	print ('  {:} sweep range:  {:}, {:}... {:} {:}'.format(set_label.title(), xmin, xstep, xmax, set_units))
	
	
	
	n_ch = len(channels)
	
	# Columns per multi-print
	n_cols = 4

	# Setup store-restore workflow
	def store(command, storage_vec, reset, reset_val):
		
		for i in range(n_ch):
			ch = channels[i]
			storage_vec[i] = qontroller.issue_command(command, ch=ch, operator='?', n_lines_requested = 1)[0][0]
			if reset:
				result = qontroller.issue_command(command, ch=ch, operator='=', value=reset_val)
		if reset:
			print ('  Will restore existing {:} values after sweeping:'.format(command))
		else:
			print ('  Using existing {:} values:'.format(command))
		
		if len("{:}".format(storage_vec[0])) > 15:
			n_cols = 2
		else:
			n_cols = 4
		
		for i in range(int(n_ch/n_cols)):
			sys.stdout.write('     ')
			for j in range(n_cols):
				sys.stdout.write(' {:};'.format(storage_vec[ n_cols*i + j ] ) )
			sys.stdout.write('\n')
		
	def restore(command, storage_vec):
		for i in range(n_ch):
			ch = channels[i]
			qontroller.issue_command(command, ch=ch, operator='=', 
									value=storage_vec[i].strip("V mA"))
		print ('  Restored previous {:} values.'.format(command))
	
	# Store calibration values
	icals = [None]*n_ch
	vcals = [None]*n_ch
	
	store(command = 'ICAL', storage_vec = icals, reset = rawi, reset_val = "0,0,0,0")
	store(command = 'VCAL', storage_vec = vcals, reset = rawv, reset_val = "0,0,0,0")
	
	# Store maxima
	imaxs = [None]*n_ch
	vmaxs = [None]*n_ch
	store(command = 'IMAX', storage_vec = imaxs, reset = ignore_limits, reset_val = qontroller.ifull)
	store(command = 'VMAX', storage_vec = vmaxs, reset = ignore_limits, reset_val = qontroller.vfull)
	
	
	
	set_xs = numpy.arange(xmin, xmax+xstep, xstep)
    #print(set_xs)
	
	# Prepare data destinations
	measured_vs = numpy.zeros_like(set_xs, dtype='f')
	measured_us = numpy.zeros_like(set_xs, dtype='f')
	measured_is = numpy.zeros_like(set_xs, dtype='f')
	
	slopes = numpy.zeros_like(channels, dtype='f')
	offsets = numpy.zeros_like(channels, dtype='f')
	errors = numpy.zeros_like(channels, dtype='f')
	
	all_var_slopes = numpy.array([[0.0]*8]*len(measured_variables))
	all_var_offsets = numpy.zeros_like(all_var_slopes, dtype='f')
	
	plot_vs = []
	plot_us = []
	plot_is = []

	if filename == '':
		# If no filename was provided write our data to a temporary file for convenience
		# This file is automatically deleted when Python is closed
		import tempfile
		file = tempfile.NamedTemporaryFile(mode='w', encoding='ascii')
	else:
		fn = filename.format(id = qontroller.device_id)
		file = open(os.path.expanduser(fn+'.csv'), 'w')
	
	file.write(time.asctime()+'\n')
	file.write('Device ID, '+str(qontroller.device_id)+'\n')
	
	file.write('Voltage calibration')
	for i in range(n_ch):
		if rawv:
			file.write(', '+str(0))
		else:
			file.write(', '+str(vcals[i]))
	file.write('\n')
		
	file.write('Current calibration')
	for i in range(n_ch):
		if rawi:
			file.write(', '+str(0))
		else:
			file.write(', '+str(icals[i]))
	file.write('\n')
	
	if not random_order:
		file.write('Set {:} ({:}),'.format(set_label, set_units))
		for x in set_xs:
			file.write(str(float(x)))
			file.write(',')
		file.write('\n')
	
	for j in range(n_ch):
		
		failed = True
		failures = 0
		
		while failed and failures < failure_repeats:
			ch = channels[j]
			if scan_keithley:
				keithley.set_scanner_channel(ch+1)
				
			# Randomise order
			if random_order:
				random.shuffle(set_xs)
		
			for i in range(len(set_xs)):
			
				# Try max 3 times before failing
				for t in range(3):
					try:
						x = set_xs[i]
						if set_variable == 'V':
							qontroller.v[ch] = float(x)
						elif set_variable == 'I':
							qontroller.i[ch] = float(x)
			
						loop_start_time = time.time()
						nap(dwell_t)
			
						if (time.time() - loop_start_time) > 10*(dwell_t+0.05):
							sys.stdout.write_to_screen_only(" Took long time to iterate!!\n")
			
						if 'V' in measured_variables:
							while True:
								try:
									measured_vs[i] = keithley.measure()
								except Exception as e:
									pass
								break
						if 'U' in measured_variables:
							measured_us[i] = qontroller.v[ch]
						if 'I' in measured_variables:
							measured_is[i] = qontroller.i[ch]
			
						if random_order:
							sys.stdout.write('  Channel {:} complete:  {:6.1f} %\r'.format(ch, 100*float(i)/len(set_xs)) )
						else:
							sys.stdout.write_to_screen_only('  Channel {:}:  {:}_set = {:.3f} {:}'.format(ch, set_variable, float(x), set_units) )
							if 'V' in measured_variables:
								sys.stdout.write_to_screen_only('; V = {:.3f} V'.format(measured_vs[i]) )
							if 'U' in measured_variables:
								sys.stdout.write_to_screen_only('; U = {:.3f} V'.format(measured_us[i]) )
							if 'I' in measured_variables:
								sys.stdout.write_to_screen_only('; I = {:.3f} mA'.format(measured_is[i]) )
							sys.stdout.write_to_screen_only('   \r')
						sys.stdout.flush()
						# We succeeded so break out of retry loop
						break
					
					except Exception as e:
						sys.stdout.write('  Encountered error: {:}.\n'.format(e) )
						status = 'retrying' if t < 2 else 'aborting'
						sys.stdout.write('  Channel {:} failed, {:}.\r'.format(ch, status) )
						measured_vs[i] = None
						measured_us[i] = None
						measured_is[i] = None
			
			# Zero the output once the ramp is complete
			if set_variable == 'V':
				qontroller.v[ch] = 0.0
			elif set_variable == 'I':
				qontroller.i[ch] = 0.0
			nap(dwell_t)
			
			# Write some ascii art
			for _ in range(80):
				sys.stdout.write_to_screen_only(' ')
			sys.stdout.write_to_screen_only('\r')
			sys.stdout.flush()
		
			# Record
			if random_order:
				file.write('Channel {:} set {:} ({:}),'.format(ch, set_label, set_units) )
				for i in range(len(set_xs)):
					file.write(str(set_xs[i]))
					file.write(',')
			file.write('\n')
			
			if 'V' in measured_variables:
				file.write('Channel {:} measured voltage (V),'.format(ch) )
				for i in range(len(set_xs)):
					file.write(str(measured_vs[i]))
					file.write(',')
			file.write('\n')
		
			if 'U' in measured_variables:
				file.write('Channel {:} on-board measured voltage (V),'.format(ch) )
				for i in range(len(set_xs)):
					file.write(str(measured_us[i]))
					file.write(',')
			file.write('\n')
		
			if 'I' in measured_variables:
				file.write('Channel {:} on-board measured current (V),'.format(ch) )
				for i in range(len(set_xs)):
					file.write(str(measured_is[i]))
					file.write(',')
			file.write('\n')
			
			# Iterate through target measured variables
			for y,iy in zip(measured_variables,range(len(measured_variables))):
				if y == 'V':
					measured_ys = measured_vs
					label = 'voltage'
					units = 'V'
				if y == 'U':
					measured_ys = measured_us
					label = 'voltage'
					units = 'V'
				if y == 'I':
					measured_ys = measured_is
					label = 'current'
					units = 'mA'
			
				# Sort data (to handle randomised input)
				data = [(copy.copy(set_xs[i]), copy.copy(measured_ys[i])) for i in range(len(set_xs))]
				data.sort()
				set_xs_s = numpy.array([d[0] for d in data])
				measured_ys_s = numpy.array([d[1] for d in data])
			
				# Fit it with a line
				(m,b) = pylab.polyfit(set_xs_s, measured_ys_s, 1)
		
				# Get uncertainty about line
				e = math.sqrt(sum([(measured_ys_s[i] - (m*set_xs_s[i] + b))**2 for i in range(len(set_xs_s))]) / len(set_xs_s))
				
				# Store
				slopes[j] = m
				offsets[j] = b
				errors[j] = e
				
				all_var_slopes[iy][j] = m
				all_var_offsets[iy][j] = b
		
				if plot:
					if plot_deviation:
						model_ys = set_xs_s * m + b
						measured_ys_s = measured_ys_s - model_ys
					else:
						measured_ys_s = measured_ys_s
					
					if y == 'V':
						plot_vs.append((copy.deepcopy(set_xs_s), copy.deepcopy(measured_ys_s)))
					if y == 'U':
						plot_us.append((copy.deepcopy(set_xs_s), copy.deepcopy(measured_ys_s)))
					if y == 'I':
						plot_is.append((copy.deepcopy(set_xs_s), copy.deepcopy(measured_ys_s)))
			
				# Check slope is in range
				tol = measured_slope_tolerance_ranges[iy]
				min_tol = tol[0] if tol != () else -float('inf')
				max_tol = tol[1] if tol != () else +float('inf')
				if tol == ():
					err_m_style = normal_text
				elif min_tol < m and m < max_tol:
					err_m_style = good_text
				else:
					err_m_style = bad_text
					qcpass = False
					
				# Check offset is in range
				tol = measured_offset_tolerance_ranges[iy]
				min_tol = tol[0] if tol != () else -float('inf')
				max_tol = tol[1] if tol != () else +float('inf')
				if tol == ():
					err_b_style = normal_text
				elif min_tol < b and b < max_tol:
					err_b_style = good_text
				else:
					err_b_style = bad_text
					qcpass = False
					
					
			
				# Print data
				if iy == 0:
					if failures:
						sys.stdout.write("    Check {ch}:  ".format(ch = ch) )
					else:
						sys.stdout.write("  Channel {ch}:  ".format(ch = ch) )
				else:
					sys.stdout.write("              ")
				sys.stdout.write("{y}_act = {e0}{m:.4f}{e1} * {x}_set {s} {e2}{b:.4f}{e1}".format(y = y, x = set_variable, m = m, s = '+' if b>0 else '-', b = abs(b),
						e0 = err_m_style,
						e1 = normal_text,
						e2 = err_b_style) )
				if print_uncertainties:
					sys.stdout.write(" +/- {:.4f}".format(e))
				sys.stdout.write("\n")
				sys.stdout.flush()
			
				# Check for failure
				if qcpass:
					failed = False
				else:
					failures += 1
					failed = True
					# Sometimes the Keithley jams up
					keithley.set_scanner_channel(0)
				
	
	
	if scan_keithley:
		keithley.set_scanner_channel(None)
	
	# Restore old calibration values
	if rawi:
		restore(command = 'ICAL', storage_vec = icals)
	
	if rawv:
		restore(command = 'VCAL', storage_vec = vcals)
	
	# Restore maxima
	restore(command = 'VMAX', storage_vec = vmaxs)
	restore(command = 'IMAX', storage_vec = imaxs)
	
	# Plot
	if plot:
		for type,data,label,u in [('V',plot_vs,"voltage","V"), ('U',plot_us,"voltage","V"), ('I',plot_is,"current","mA")]:
			if type in measured_variables:
				for set,trace in data:
					pylab.plot(set, trace)
				pylab.xlabel('{:} ({:})'.format(set_label.title(), set_units))
				pylab.ylabel('Measured {:}{:} ({:})'.format(label," deviation" if plot_deviation else "", u))
				if filename != '':
					pylab.savefig(fn+' '+type+'-'+set_variable+'.pdf', bbox_inches='tight')
				#pylab.show()
				pylab.close('all')
	
	
	
	file.close()
	
	
	# If user requested more than one measurement, give matrix of slopes and offsets
	if len(measured_variables) > 1:
		return (all_var_slopes, all_var_offsets, qcpass)
	else:
		return (slopes, offsets, qcpass)



def get_new_device_id(
		id_prefix = None, # None means increment globally; prefix means go from nearest same type
		target_device_id = None, # None means automatically selected new ID
		record_filename = '~/Qontrol Dropbox/Engineering/Outgoing devices/Programmed devices.csv',
		preexisting_ids = [] # List of IDs not in file to treat as taken
		):
	
	regex = re.compile("\w+-([A-F0-9]+)")
	
	def num_of_id(id):
		return int(regex.search(id).groups()[0], 16)
	
	# Capture preexisting IDs and corresponding numbers
	existing_ids = copy.deepcopy(preexisting_ids)
	existing_id_nums = [num_of_id(id) for id in existing_ids]
	
	with open(os.path.expanduser(record_filename)) as file:
		for line in file:
			# Regex each line in file to find ids
			result = regex.search(line)
			if result is not None:
				for id in result.groups():
					existing_ids.append(id)
					# Store hex integer conversion
					existing_id_nums.append( int(id, 16) )
	
	# Assign ID number
	if target_device_id is None:
		# Get next unused ID number
		new_device_id_num = max(existing_id_nums) + 1
	else:
		new_device_id_num = num_of_id(target_device_id)
	
	# Construct device ID
	new_device_id = id_prefix + '-' + '{:04X}'.format(new_device_id_num)
	
	# Return
	return (new_device_id, new_device_id_num, existing_ids)
	


def create_device_id(
		qontroller,
		id_prefix = None, # None means increment globally; prefix means go from nearest same type
		hw_rev = 0,
		target_device_id = None, # None means automatically selected new ID
		customer_or_quote = 'QS000',
		record_filename = '~/Qontrol Dropbox/Engineering/Outgoing devices/Programmed devices.csv'):
	
	# Keep track of success
	qcpass = True
	
	print ('\nCreating device record...')

	print ("  Customer or quote:    {:}{:}{:}".format(emph_text, customer_or_quote, normal_text))


	# Format hardware revision
	try:
		hw_rev = '0x{:02X}'.format(hw_rev)
	except:
		pass

	print ("  Hardware revision:    {:}{:}{:}".format(emph_text, hw_rev, normal_text))

	# Get firmware revision
	firmware_rev = qontroller.issue_command ('firmware', operator='?', n_lines_requested=2**31, output_regex='v(.+)')[0][0]

	print ("  Firmware revision:    {:}{:}{:}".format(emph_text, firmware_rev, normal_text))


	# Find out which IDs already have been assigned
	(new_device_id, new_device_id_num, existing_ids) = get_new_device_id(
														id_prefix, 
														target_device_id, 
														record_filename, 
														preexisting_ids = [])
	
	# Set it
	result = qontroller.issue_command ('id', operator='=', value = new_device_id_num)

	# Read it back
	new_device_id = qontroller.issue_command ('id', operator='?')[0][0]
	
	# Update internal device ID
	qontroller.device_id = new_device_id

	print ("  Using new device ID:  {:}{:}{:}".format(emph_text, new_device_id, normal_text))
	
	new_id_unique_q = new_device_id not in existing_ids
	if new_id_unique_q:
		status = "{:}{:}{:}".format(emph_text, 'True', normal_text)
	else:
		status = "{:}{:}{:}".format(bad_text, 'False', normal_text)
		qcpass = False
	print ('  New ID is unique:     {:}'.format(status))

	# Get current date and time
	date_programmed = time.asctime()

	# Ensure newline at end of database is handled properly
	with open(os.path.expanduser(record_filename), 'rb+') as f:
		f.seek(-1,2)
		last_char = f.read()

	file = open(os.path.expanduser(record_filename), 'a')
	if last_char != '\n':
		file.write('\n')

	# Add new record for this device
	new_device_record = [new_device_id, date_programmed, firmware_rev, hw_rev, customer_or_quote]

	file.write(new_device_record[0])
	for i in new_device_record[1:]:
		file.write(', ' + str(i))
		
	return qcpass


def safe_rename(source_filename, dest_filename):
	# Rename temporary files
	
	if not os.path.isfile(source_filename):
		# print ("  "+source_filename+" has not been created")
		return
	dfn = dest_filename
	fni = 1
	while os.path.isfile(dfn):
		l = dest_filename.split('.')
		dfn = '.'.join(l[:-1])+' '+str(fni)+'.'+l[-1]
		fni += 1
	os.rename(source_filename, dfn)
	print ("  "+dfn+" created")
	return

def calibrate_voltage(q, timeout = 4.000):
	"""
	Prompt device to calibrate _its own_ voltage
	
	Q8: calibrate output voltage
	Q8iv: calibrate input voltage
	"""
	
	print ('\nCalibrating on-board voltage...')
	result = q.issue_command('safe', ch=None, operator='=', value='0', n_lines_requested = 1)
	for ch in range(8):
		result = q.issue_command('vcal', ch=ch, special_timeout = timeout)
		result = q.issue_command('vcal', ch=ch, operator='?', n_lines_requested = 1)[0][0]




def calibrate_current(q, vstep = 0.2, vmax = 20.0, filename = ''):
	
# 	q.vmax[:] = q.v_full
	
	qcpass = True
	
	(ms, bs, qc) = get_XY(
		q, 
		keithley = None,
		scan_keithley = False,
		xmin = 0.0,
		xmax = vmax, 
		xstep = vstep, 
		set_variable = 'v',
		measured_variables = ['i'],
		measured_slope_tolerance_ranges = [],
		filename = '', 
		plot = True,
		rawi = True,
		rawv = False,
		random_order = False,
		dwell_t = 0.1,
		channels = range(8))
	
	qcpass &= qc


	print ('\nUpdating current calibration...')

	for ch in range(8):
		ical = int(- (ms[ch] - 1) * 2 * 0x7FFF)
		result = q.issue_command('ical', ch=ch, operator='=', value=ical, n_lines_requested = 1)
	
	(_,_,qc) = get_XY(
		q, 
		keithley = None,
		scan_keithley = False,
		xmin = 0.0,
		xmax = vmax, 
		xstep = vstep, 
		set_variable = 'v',
		measured_variables = ['i'],
		measured_slope_tolerance_ranges = [(0.99,1.01)],
		filename = filename, 
		plot = True,
		rawi = False,
		rawv = False,
		random_order = False,
		dwell_t = 0.1,
		channels = range(8))
		
	qcpass &= qc
	
	return qcpass
			


def calibrate_output_voltage(
		qontroller, 
		keithley, 
		vstep = 0.2, 
		vmax = 12.0, 
		filename = ''):
	
	"""
	Use external voltmeter to calibrate voltage
	"""
	
	print("Calibrating output voltage...")
	
	qcpass = True
	
	qontroller.vmax[:] = qontroller.v_full
	
	(ms, bs, qc) = get_XY(
		qontroller, 
		keithley,
		xmin = 0.0,
		xmax = vmax, 
		xstep = vstep, 
		set_variable = 'v',
		measured_variables = ['v'],
		measured_slope_tolerance_ranges = [],
		measured_offset_tolerance_ranges = [],
		filename = '', 
		plot = True,
		rawi = False,
		rawv = True,
		random_order = False,
		dwell_t = 0,
		channels = range(8),
		scan_keithley = True)
	
	qcpass &= qc

	print ('\nUpdating voltage output calibration...')

	for ch in range(8):
		vcal_m = int(- (ms[ch] - 1) * 2 * 0x7FFF)
		vcal_b = int(- bs[ch] * 0xFFFF / 12.0)
		result = qontroller.issue_command('vcal', ch=ch, operator='=', value = '0,0,{m},{b}'.format(m=vcal_m, b=vcal_b), n_lines_requested = 1)
	
	
	(_,_,qc) = get_XY(
		qontroller, 
		keithley,
		xmin = 0.0,
		xmax = vmax, 
		xstep = vstep, 
		set_variable = 'v',
		measured_variables = ['v'],
		measured_slope_tolerance_ranges = [(0.99,1.01)],
		measured_offset_tolerance_ranges = [(-0.1,0.1)],
		filename = filename, 
		plot = True,
		rawi = False,
		rawv = False,
		random_order = False,
		failure_repeats = 2,
		dwell_t = 0,
		channels = range(8),
		scan_keithley = True)
		
	qcpass &= qc
	
	return qcpass



def calibrate_input_current_with_R(
		qontroller, 
		keithley, 
		resistances,
		ignore_limits = False,
		vstep = 0.2, 
		vmax = 12.0, 
		n_chs = 8,
		dwell_t = 0.05,
		filename = ''):
	
	"""
	Use external voltmeter to calibrate voltage across known test resistances
	"""
	
	print("\nCalibrating input current using test resistances...")
	
	qcpass = True
	
	qontroller.vmax[:] = qontroller.v_full
	
	ifull = qontroller.i_full
	device_type = qontroller.chain[0]['device_type']
	
	(ms, bs, qc) = get_XY(
		qontroller, 
		keithley,
		xmin = 0.0,
		xmax = vmax, 
		xstep = vstep, 
		set_variable = 'v',
		measured_variables = ['i'],
		measured_slope_tolerance_ranges = [],
		measured_offset_tolerance_ranges = [],
		filename = filename+'_raw', 
		plot = True,
		rawi = True,
		rawv = True,
		random_order = False,
		ignore_limits = ignore_limits,
		dwell_t = dwell_t,
		channels = range(n_chs),
		scan_keithley = True)
	
	qcpass &= qc

	print ('\nUpdating current input calibration from test resistances...')

	for ch in range(n_chs):
		R = resistances[ch]/1000 # kOhm
		g = R*ms[ch]
		if device_type == 'Q8iv':
			o = bs[ch]
		elif device_type == 'Q8b':
			o = bs[ch] #1.59553 * bs[ch] - 1.76231 # This is empirical
		ical_m = int(- (g - 1) * 2 * 0x7FFF)
		ical_b = int(- o * 0xFFFF / ifull)
		
		result = qontroller.issue_command('ical', ch=ch, operator='=', value = '{m},{b},0,0'.format(m=ical_m, b=ical_b), n_lines_requested = 1)
	
	
	(ms,bs,qc) = get_XY(
		qontroller, 
		keithley,
		xmin = 0.0,
		xmax = vmax, 
		xstep = vstep, 
		set_variable = 'v',
		measured_variables = ['i'],
		measured_slope_tolerance_ranges = [],
		measured_offset_tolerance_ranges = [],
		filename = filename, 
		plot = True,
		rawi = False,
		rawv = True, # TODO: Check if this should be true or false
		random_order = False,
		ignore_limits = ignore_limits,
		failure_repeats = 2,
		dwell_t = dwell_t,
		channels = range(n_chs),
		scan_keithley = True)
	
	qcpass &= qc
	
	print ("\nCurrent input calibration state:")
	for ch in range(n_chs):
		R = resistances[ch]/1000.0 # kOhm
		pf_m = ( abs(ms[ch]*R - 1) < 0.05)
		pf_b = ( abs(bs[ch]) < 0.5) # mA
		
		qcpass &= pf_m
		qcpass &= pf_b
		try:
			print ("  Channel {:}: Slope error = {:5.2f}% {:}; offset error = {:4.2f} mA {:}".format(ch, 
						abs(ms[ch]*R - 1)*100, passfail( pf_m ) ,
						abs(bs[ch]), passfail( pf_b ) 
						) )
		except Exception as e:
			print ("Error was {:}".format(e))
			for arg in [ch, 
						abs(ms[ch]*R - 1)*100, passfail( pf_m ) ,
						abs(bs[ch]), passfail( pf_b ) ]:
				print (arg)
		
	
	return qcpass


def calibrate_output_voltage_and_input_current_with_R(
		qontroller, 
		keithley, 
		resistances,
		ignore_limits = False,
		vstep = 0.2, 
		vmax = 12.0, 
		n_chs = 8,
		dwell_t = 0.05,
		filename = ''):
	
	"""
	Use external voltmeter to calibrate voltage across known test resistances
	"""
	
	print("\nCalibrating output voltage and input current using test resistances...")
	
	qcpass = True
	
	qontroller.vmax[:] = qontroller.v_full
	
	ifull = qontroller.i_full
	device_type = qontroller.chain[0]['device_type']
	
	(ms,bs,qc) = get_XY(
		qontroller, 
		keithley,
		xmin = 0.0,
		xmax = vmax, 
		xstep = vstep, 
		set_variable = 'v',
		measured_variables = ['v','i'],
		measured_slope_tolerance_ranges = [(0.99,1.01),()],
		measured_offset_tolerance_ranges = [(-0.1,0.1),()],
		filename = filename+'_raw', 
		plot = True,
		rawi = True,
		rawv = True,
		random_order = False,
		ignore_limits = ignore_limits,
		dwell_t = dwell_t,
		channels = range(n_chs),
		scan_keithley = True)
	
	qcpass &= qc
	
	vms = ms[0]
	ims = ms[1]
	vbs = bs[0]
	ibs = bs[1]
	
	print ('\nUpdating voltage output calibration...')

	for ch in range(8):
		vcal_m = int(- (vms[ch] - 1) * 2 * 0x7FFF)
		vcal_b = int(- vbs[ch] * 0xFFFF / 12.0)
		result = qontroller.issue_command('vcal', ch=ch, operator='=', value = '0,0,{m},{b}'.format(m=vcal_m, b=vcal_b), n_lines_requested = 1)


	print ('\nUpdating current input calibration from test resistances...')

	for ch in range(n_chs):
		R = resistances[ch]/1000 # kOhm
		g = R*ims[ch]
		if device_type == 'Q8iv':
			o = ibs[ch]
		elif device_type == 'Q8b':
			o = ibs[ch] #1.59553 * bs[ch] - 1.76231 # This is empirical
		ical_m = int(- (g - 1) * 2 * 0x7FFF)
		ical_b = int(- o * 0xFFFF / ifull)
		
		result = qontroller.issue_command('ical', ch=ch, operator='=', value = '{m},{b},0,0'.format(m=ical_m, b=ical_b), n_lines_requested = 1)
	
	
	(ms,bs,qc) = get_XY(
		qontroller, 
		keithley,
		xmin = 0.0,
		xmax = vmax, 
		xstep = vstep, 
		set_variable = 'v',
		measured_variables = ['v','i'],
		measured_slope_tolerance_ranges = [],
		measured_offset_tolerance_ranges = [],
		filename = filename, 
		plot = True,
		rawi = False,
		rawv = False,
		random_order = False,
		ignore_limits = ignore_limits,
		failure_repeats = 2,
		dwell_t = dwell_t,
		channels = range(n_chs),
		scan_keithley = True)
	
	vms = ms[0]
	ims = ms[1]
	vbs = bs[0]
	ibs = bs[1]
	
	qcpass &= qc
	
	print ("\nCurrent input calibration results:")
	for ch in range(n_chs):
		R = resistances[ch]/1000 # kOhm
		pf_m = ( abs(ims[ch]*R - 1) < 0.05)
		pf_b = ( abs(ibs[ch]) < 0.5) # mA
		qcpass &= pf_m
		qcpass &= pf_b
		print ("  Channel {:}: Slope error = {:.2f}% {:}; offset error = {:.2f} mA {:}".format(ch, 
					abs(ims[ch]*R - 1)*100, passfail( pf_m ) ,
					abs(ibs[ch]), passfail( pf_b ) 
					) )
		
	
	return qcpass
		
	

def obtain_adc_speed_and_err(q, filename, channels = [0], adcts = [1,8,16,24,31], adcns = [1,2,4,8,16,32,64,128], dts = [0,0.01,0.02,0.03,0.04,0.05,0.1,0.15,0.2,0.25,0.3,0.35,0.4,0.5,1.0,1.5,2.0,3.0], vs = [5,10,20], rawv = False, rawi = True, t_settle = 3.0, t_adcnt_settle = 5.0, err_samples = 100):

	print ('Predicted completion time: {:} s'.format(len(channels)*len(adcts)*len(adcns)*len(vs)*(len(dts)*t_settle + sum(dts) ) + len(adcts)*len(adcns)*t_adcnt_settle ) )


	start_time = time.time()

	speed_fn = filename.format(id = q.device_id, purpose = 'SPEED')
	
	err_fn = filename.format(id = q.device_id, purpose = 'UNCERTAINTY')
	
	
	file_speed = open(os.path.expanduser(speed_fn), 'w')
	file_err = open(os.path.expanduser(err_fn), 'w')
	
	def fwrite(s, targ = None):
		if targ is None:
			file_speed.write(s)
			file_err.write(s)
		elif targ == 'speed':
			file_speed.write(s)
		elif targ == 'err':
			file_err.write(s)
		else:
			raise RuntimeError('No file')
		
	

	fwrite(time.asctime()+'\n')
	fwrite('Device ID, '+str(q.device_id)+'\n')
	fwrite('Firmware, '+str(q.issue_command('firmware', None, '?', 1, "(.*)")[0][0])+'\n')


	# Get calibration values 
	n_ch = len(channels)
	icals = range(n_ch)
	vcals = range(n_ch)

	for j in range(n_ch):
		ch = channels[j]
		icals[j] = int(q.issue_command('ical', ch=ch, operator='?', n_lines_requested = 1)[0][0])
		if rawi:
			result = q.issue_command('ical', ch=ch, operator='=', value=0)
		
	if rawi:
		print ('  Stored existing ICAL values:  {:}'.format(icals))
	else:
		print ('  Existing ICAL values:  {:}'.format(icals))
	
	
	is_vcal = not (int(q.issue_command('vcal', ch=channels[0], operator='?', n_lines_requested = 1)[0][0]) == '0')
	
	if not is_vcal:
		calibrate_voltage(q, timeout = 4.000)
		
	
	for j in range(n_ch):
		ch = channels[j]
		vcals[j] = int(q.issue_command('vcal', ch=ch, operator='?', n_lines_requested = 1)[0][0])
		if rawv:
			result = q.issue_command('vcal', ch=ch, operator='=', value=0)




	fwrite('Voltage calibration')
	for j in range(n_ch):
		ch = channels[j]
		if rawv:
			fwrite(', '+str(0))
		else:
			fwrite(', '+str(vcals[j]))
	fwrite('\n')

	fwrite('Current calibration')
	for j in range(n_ch):
		ch = channels[j]
		if rawi:
			fwrite(', '+str(0))
		else:
			fwrite(', '+str(icals[j]))
	fwrite('\n')


	fwrite('Channel Number,')

	fwrite('ADCT,')
	
	fwrite('ADCN,')

	fwrite('Voltage (v),')

	# Speed file keeps track of time
	fwrite('Time (s),', 'speed')
	for t in dts:
		fwrite(str(float(t)), 'speed')
		fwrite(',', 'speed')
		
	# Err file keeps track of uncertainty
	fwrite('Current reading (mA)', 'err')

	for ch in channels:
		for t in adcts:
			q.set_value(ch=None, para = 'adct',new = t)
			for n in adcns:
				q.set_value(ch=None, para = 'adcn',new = n)
				print ('Waiting for ADC to settle (ADCT = {:}, ADCN = {:})'.format(t,n))
				nap(t_adcnt_settle)
				for v in vs:
				
					fwrite('\n')
				
					fwrite(str(ch)+',')

					fwrite(str(t)+',')
	
					fwrite(str(n)+',')
				
					fwrite(str(v)+',')
				
					fwrite(',')
				
					for dt in dts:
					
						q.v[ch] = 0.0
						nap(t_settle)
						q.v[ch] = float(v)
						nap(dt)
						current = q.i[ch]
						fwrite(str(current)+',', 'speed')
						sys.stdout.write ('ch:{:5} v:{:5} t:{:5} = i:{:5}'.format(ch,v,dt,current))
					
					nap(t_settle)
					idata=[]
					for meas in range(err_samples):
						current = q.i[ch]
						idata.append(current)
						fwrite(str(current)+',', 'err')
						nap(0.05)
					mean=sum(idata)/err_samples
					stddev=math.sqrt(sum([(mean-id)**2 for id in idata])/err_samples)
					print (', di:{:6.3}+/-{:5.3}\n'.format(mean,stddev))

	file_speed.close()
	file_err.close()

	print ("Took {:} s to complete.".format(time.time() - start_time))



def stability_measurement(q, keithley, filename, channels = [7], voltage = 20.0):


	fn = filename.format(id = q.device_id)
	
	
	file = open(os.path.expanduser(fn), 'w')
	

	
	file.write(time.asctime()+'\n')
	file.write('Device ID, '+str(q.device_id)+'\n')
	file.write('Firmware, '+str(q.issue_command('firmware', None, '?', 1, "(.*)")[0][0])+'\n')

	# Get calibration values 
	n_ch = len(channels)
	icals = range(n_ch)
	vcals = range(n_ch)

	for j in range(n_ch):
		ch = channels[j]
		icals[j] = int(q.issue_command('ical', ch=ch, operator='?', n_lines_requested = 1)[0][0])
		
	print ('  Existing ICAL values:  {:}'.format(icals))
	
	
	is_vcal = not (int(q.issue_command('vcal', ch=channels[0], operator='?', n_lines_requested = 1)[0][0]) == '0')
	
	if not is_vcal:
		calibrate_voltage(q, timeout = 4.000)
	
	for j in range(n_ch):
		ch = channels[j]
		vcals[j] = int(q.issue_command('vcal', ch=ch, operator='?', n_lines_requested = 1)[0][0])
		
	print ('  Existing VCAL values:  {:}'.format(vcals))

	file.write('Voltage calibration')
	for j in range(n_ch):
		file.write(', '+str(vcals[j]))
	file.write('\n')

	file.write('Current calibration')
	for j in range(n_ch):
		file.write(', '+str(icals[j]))
	file.write('\n')
	
	file.write('ADCT')
	for j in range(n_ch):
		ch = channels[j]
		file.write(', '+q.issue_command('adct', ch=ch, operator='?', n_lines_requested = 1)[0][0])
	file.write('\n')
	
	file.write('ADCN')
	for j in range(n_ch):
		ch = channels[j]
		file.write(', '+q.issue_command('ADCN', ch=ch, operator='?', n_lines_requested = 1)[0][0])
	file.write('\n')
	
	file.write('Voltage setpoint (V)')
	for j in range(n_ch):
		file.write(', '+str(voltage))
	file.write('\n')
	


	file.write('Channel Number,')

	file.write('Time (s),')

	file.write('Measured voltage (v),')
	
	file.write('ADC current (v)')
	
	# Set voltage on channels
	for ch in channels:
		q.v[ch] = float(voltage)
	
	# Setup Keithley
	keithley.set_integration_time(10.0) # 10 power line cycles
	print ("Waiting for Keithley to settle...")
	
	nap (3.0)
	
	start_time = time.time()
	
	print ("Measuring stability. Use Ctrl-c to stop.")
	
	try:
		# Cycle through channels, measuring once per second
		while (True):
			for ch in channels:
		
				nap(1.0)
		
				t = time.time() - start_time
		
				v = keithley.measure()
		
				i = q.i[ch]
				
				sys.stdout.write("Elapsed time: {t:6} s. Voltage error: {v:6.3} V. Current error: {i:6.3} mA".format(t = round(time.time()-start_time), v = v-voltage, i = i-voltage))
				for s in range(10):
					sys.stdout.write(" ")
				sys.stdout.write("\r")
				sys.stdout.flush()
		
				file.write('\n')
	
				file.write(str(ch)+',')

				file.write(str(t)+',')

				file.write(str(v)+',')
	
				file.write(str(i))
				
				file.flush()
	except KeyboardInterrupt():
		file.close()
		print ("Took {:} s to complete.".format(time.time() - start_time))



def nvm_parameters(q):
	param_dict = {}
	def get_q_str(cmd):
		return q.issue_command (cmd, operator='?', output_regex='(.+)', n_lines_requested = 1)[0][0]
	def get_q_int(cmd):
		return int(q.issue_command (cmd, operator='?', output_regex='([-+0-9.,]+)', n_lines_requested = 1)[0][0])
	def get_all_q_int(cmd, op=int):
		xs = []
		for i in range(8):
			# print ('Getting '+cmd+str(i)+'...')
			xs.append(q.issue_command (cmd, ch=i, operator='?',output_regex='([-+0-9.,]+)', n_lines_requested = 1) )
		return [op(xs[i][0][0]) for i in range(8)]
	def round_float(f):
		return round(float(f),1)
	def get_q_dev_id_num():
		ob = re.match('(.+)-([0-9a-fA-F\*]+)', str(get_q_str('id')))
		if ob is None:
			return None
		else:
			dev_type,dev_num = ob.groups()
			return int(dev_num, 16)

	# Get NVM parameters. These are restored later
	param_dict.update( {'ID':get_q_str('id')} )
	
	param_dict.update( {'ID_INT':get_q_dev_id_num()} )
	
	try:
		param_dict.update( {'ADCT':get_q_int ('adct')} )
	except:
		print ("Warning: Failed to get ADCT from NVM")
		param_dict.update( {'ADCT':None} )
	
	param_dict.update( {'ADCN':get_q_int ('adcn')} )
	
	param_dict.update( {'Lifetime':get_q_int ('lifetime')} )
	
	param_dict.update( {'VCAL':get_all_q_int ('vcal', op=str)} )
	
	param_dict.update( {'ICAL':get_all_q_int ('ical', op=str)} )
	
	
	param_dict.update( {'VMAX':get_all_q_int ('vmax', op=round_float)} )
	
	param_dict.update( {'IMAX':get_all_q_int ('imax', op=round_float)} )
	
	return param_dict



def setup_keithley(serial_port_glob = '/dev/tty.usbserial-A*'):
	
	errored = False
	
	while True:
		try:
			serial_ports = glob.glob(serial_port_glob)
	
			if len(serial_ports) > 1:
				raise RuntimeError('Found multiple potential serial ports. Specify the correct one manually.')
			else:
				usb = serial_ports[0]
	
			k2000 = Keithley2000(
						serial_port_name = usb,
						serial_timeout = 0.5)
			
			if k2000.device_id in ["", None]:
				raise RuntimeError()
	
			print ("\nInitialised multimeter\n  ID: '{:}'\n  Serial port: '{:}'".format(k2000.device_id.strip(), usb))

			k2000.check_error()
		
			return k2000
		
		except RuntimeError:
			if not errored:
				print("{:}Error{:}: Failed to initialise Keithley. Is it turned on and connected? Retrying...".format(bad_text,normal_text))
				errored = True
				nap(5.0)
				
				

def setup_psu(serial_port_glob = '/dev/tty.usbserial-14130'):
	
	errored = False
	
	while True:
		try:
			serial_ports = glob.glob(serial_port_glob)
	
			if len(serial_ports) > 1:
				raise RuntimeError('Found multiple potential serial ports. Specify the correct one manually.')
			else:
				usb = serial_ports[0]
	
			psu = Keithley2231A(
						serial_port_name = usb,
						serial_timeout = 0.5)
			
			if psu.device_id in ["", None]:
				raise RuntimeError()
	
			print ("\nInitialised DC power supply\n  ID: '{:}'\n  Serial port: '{:}'".format(psu.device_id.strip(), usb))
		
			return psu
		
		except RuntimeError:
			if not errored:
				print("{:}Error{:}: Failed to initialise DC power supply Keithley 2231A. Is it turned on and connected? Retrying...".format(bad_text,normal_text))
				errored = True
				nap(5.0)



def programme_and_log(customer_or_quote = 'Customer', 
			filename_prefix = '~/Qontrol Dropbox/Engineering/Outgoing devices/', 
			flash_script_filename = '~/Qontrol Dropbox/Code/Firmware/qontrol_firmware.X/flash.sh', 
			config_filename = '~/Qontrol Dropbox/Code/Firmware/qontrol_firmware.X/src/config.h', 
			record_filename = 'Programmed devices.csv', 
			force_preserve_nvm = False,
			force_id = None,			# Force a particular ID on it
			calibrate_v = True,			# Calibrate the new Qontroller
			calibrate_i = True,			# Calibrate the new Qontroller
			high_quality_iv = True,		# Get high-quality I-V curve for records
			name = True,				# Name the new Qontroller
			device_type = 'q8iv',		# Type of module (e.g. q8iv) matching git branch 
			test_resistances = None,	# Values of test resistances(None for unspecified)
			qontroller_serial_port_glob = '/dev/tty.usbserial-FT*',
			flash_it = True,
			loop = True,
			psu = None					# PSU handle (optional)
			):
	
	"""
	Program a new module, store its details, generate a report.
	
	filename_prefix:		directory containing record_filename and for report destination
	flash_script_filename:	path to flash script ('flash.sh')
	config_filename:		path to config build file with build info ('config.h')
	record_filename:		filename (no path) of csv device database
	force_preserve_nvm:		whether to record and not overwrite memory (e.g. calibration)
	force_id:				string (e.g. "Q8-00A1")
	qontroller_serial...:	glob (including e.g. *) to deduce serial port
	flash_it:				Flag to optionally not flash the device (for debugging)
	test_resistances:		Values of test resistances for current cal (None for unspecified)
	"""
	
	
	response = 'y'

	print ("Starting...")
	
	nap(2.0)
	
	os.chdir(os.path.expanduser(filename_prefix))
	
	# Find qontroller serial port
	serial_ports = glob.glob(qontroller_serial_port_glob)
	if len(serial_ports) > 1:
		raise RuntimeError('Found multiple potential serial ports. Specify the correct one manually. Ports are {:}'.format(serial_ports))
	else:
		serial_port_name = serial_ports[0]
	
	# Allow default force_id to be either None or False
	if force_id == False:
		force_id = None
	
	
	# Loop while user wants to keep programming
	while response == 'y':

		t_start = time.time()
		
		# Copy stdout to log file
		sys.stdout = Tee('Temporary programming log.txt', 'w')
		
		
		#########################
		### ESTABLISH REALITY ###
		#########################
		
		# Check if this device already has an ID, store it
		qcpass_id = True
		print ("\nChecking whether device is already programmed...")
		try:
			q = setup_qontroller(
					serial_port_name = serial_port_name,
					timeout = 0.5)
			id = q.device_id
			try:
				ob = re.match('.+-([0-9a-fA-F]+)', id)
			except:
				ob = None
			
			if ob is not None:
				print ("  Using existing ID: {:}{:}{:}".format(emph_text,id,normal_text))
				
				print ("  Firmware revision: {:}{:}{:}".format(emph_text,q.issue_command ('firmware', operator='?', n_lines_requested = 1)[0][0], normal_text) )
			else:
				id = None
				if name:
					print ("  No existing ID. Will create.")
				if force_preserve_nvm and not name:
					print ("  Device has no ID, aborting "+bad_text+"[ERROR]"+normal_text)
					qcpass_id = False
					if loop:
						response = raw_input('Program another device? (y/n) : ')
					continue
		except RuntimeError as e:
			id = None
			nvmall = None
			if name:
				print ("  Not programmed. Will use new ID.")
			if (force_preserve_nvm):
				print ("  Failed to setup Qontroller, forcing NVM preservation.\n  Aborting as a consequence.\n  Error was '{:}'.".format(e))
				qcpass_id = False
				qcpass_cal = False
				break
		
			
		if force_id is not None and name:
			if id is not None and id != force_id:
				qcpass_id = False
				print ("  Overwriting existing ID, {:}{:}{:}".format(bad_text, id, normal_text) )
			id = force_id
			print ("  Forcing ID: {:}".format(id))
		
		
		if force_preserve_nvm:
			# Get NVM parameters. These are restored later
			print ("\nGetting pre-flash NVM parameters...")
			preflight_nvmparams = nvm_parameters(q)
			
			sys.stdout.write_to_file_only("  Pre-flight parameters: ")
			sys.stdout.write_to_file_only( str(preflight_nvmparams) + "\n")
			
			q.close()
		
		
		########################
		### PROGRAM and NAME ###
		########################
		
		# Flash it
		if flash_it:
			err = program_device(flash_script_filename = flash_script_filename, device_type = device_type)
		else:
			err = 0

		if err == 1:
			raise RuntimeError('Failed to compile firmware.');
		elif err == 2:
			raise RuntimeError('Failed to program device.');

		# Reconnect to device
		try:
		
			q = setup_qontroller(
				serial_port_name = serial_port_name,
				timeout = 0.2)
		except RuntimeError:
			raise RuntimeError("Device is not communicating after programming. Module hardware issue?")
		
		
		
		# Get compiled device properties from config.h
		with open(os.path.expanduser(config_filename),"r") as f:
			file = f.read()
			# Get firmware version
			m = re.search("^\s*#define FIRMWARE_VERSION_STR \"(.*)\"", file, re.MULTILINE)
			try:
				firmware = m.groups()[0]
			except:
				firmware = 'Unknown'
			# Get device type
			m = re.search("^\s*#define TARGET_(.*)", file, re.MULTILINE)
			try:
				id_prefix = m.groups()[0]
			except:
				id_prefix = 'Unknown'
			# Get hardware revision
			m = re.search("^\s*#define HW_REV (.*)", file, re.MULTILINE)
			try:
				hw_rev = int(m.groups()[0])
			except:
				hw_rev = 'Unknown'
		
		
		# Generate ID record in database
		if name:
			qcpass_id &= create_device_id(
					q,
					id_prefix = id_prefix,
					target_device_id = id,
					hw_rev = hw_rev,
					customer_or_quote = customer_or_quote,
					record_filename = record_filename)
		
		# Get number of channels
		# TODO: this should be dynamic
		n_ch = 8
		
		
		###################
		### RESTORE NVM ###
		###################
		
		if force_preserve_nvm:
			# Flush any faulty commands
			for i in range(3):
				q.issue_command ('id', operator='?')
			# Restore previously acquired parameters
			if not name:
				q.issue_command ('id', operator='=', value=preflight_nvmparams['ID_INT'], output_regex='(OK)')
			if preflight_nvmparams['ADCT'] is not None:
				q.issue_command ('adct', operator='=', value=preflight_nvmparams['ADCT'], output_regex='(OK)')
			q.issue_command ('adcn', operator='=', value=preflight_nvmparams['ADCN'], output_regex='(OK)')
			q.issue_command ('lifetime', operator='=', value=preflight_nvmparams['Lifetime'], output_regex='(OK)')
			
			for i in range(8):
				q.issue_command ('vcal', ch=i, operator='=', value=preflight_nvmparams['VCAL'][i], output_regex='(OK)', n_lines_requested = 1)
				q.issue_command ('ical', ch=i, operator='=', value=preflight_nvmparams['ICAL'][i], output_regex='(OK)', n_lines_requested = 1)
				q.issue_command ('vmax', ch=i, operator='=', value=preflight_nvmparams['VMAX'][i], output_regex='(OK)', n_lines_requested = 1)
				q.issue_command ('imax', ch=i, operator='=', value=preflight_nvmparams['IMAX'][i], output_regex='(OK)', n_lines_requested = 1)
			
			q.wait(0)
			
			# Check them
			print ("\nGetting restored parameters...")
			
			postflight_nvmparams = nvm_parameters(q)
			
			sys.stdout.write_to_file_only("  Post-flight parameters: ")
			sys.stdout.write_to_file_only( str(postflight_nvmparams) + "\n")
			
			err = False
			for k in postflight_nvmparams.keys():
				
				if postflight_nvmparams[k] == preflight_nvmparams[k] or k == 'Lifetime':
					status = "{:}[{:}]{:}".format(good_text, 'PASS', normal_text)
					print ("  {key: <9}: {stat}".format(
							key = k,
							stat = status) )
				else:
					status = "{:}[{:}]{:}".format(bad_text, 'FAIL', normal_text)
					err = True
					print ("  {key: <9}: {stat}\n   Before {pre}, \n   After  {post}".format(
							key = k,
							pre = preflight_nvmparams[k],
							post = postflight_nvmparams[k],
							stat = status) )
			
			if err:
				print ("  Warning: Before and after NVM data do not match!")
			
			
		
		###################
		### CALIBRATION ###
		###################
		
		qcpass_cal = True
		
		if calibrate_v or calibrate_i:
			
			# Print test resistance values for record
			if test_resistances is not None:
				print ("\nTest resistance values:")
				
				n_cols = 4
		
				for i in range(int(int(n_ch/n_cols))):
					sys.stdout.write('   ')
					for j in range(n_cols):
						sys.stdout.write(' {:} Ohm;'.format(test_resistances[ n_cols*i + j - 1] ) )
					sys.stdout.write('\n')
					
				R_mean = numpy.mean(test_resistances)/1000.0 # kOhm
			else:
				print ("\nTest resistance not specified.")
				R_mean = 1.0 # kOhm
			R_mean_range = (0.9*R_mean, 1.1*R_mean)
			S_mean_range = (0.9/R_mean, 1.1/R_mean)
			
			# Q8 calibration routine
			if id_prefix == 'Q8':
				if calibrate_v:
					# Use on-board voltage self calibration
					calibrate_voltage(
						q,
						timeout = 4.0)
				if calibrate_i:
					# Calibrate current against standard (1kOhm) load
					qcpass_cal &= calibrate_current(
						q,
						vstep = 0.2,
						vmax = 20.0,
						filename = 'Temporary calibration')
			
			# Q8b calibration routine
			if id_prefix == 'Q8b':
				
				# This is 20 V for HW < 03, 12 V for HW03
				vmax = float(q.vfull[:-1])
		
				# Try to setup keithley
				keithley = setup_keithley()
				
				# Tell the Q8b to use the on-board 5V regulator for calibration
				q.issue_command('digsup', operator='=', value=0)
				print ("\nUsing on-board 5V regulator for calibration...")
				
				if high_quality_iv:
					# Sweep voltage measuring off-board voltage and on-board voltage and current
					print ("\nCollecting high-quality raw I-V curves...\n")
					
					(_,_,qc) = get_XY(
						q, 
						keithley,
						xmin = 0.0,
						xmax = vmax, 
						xstep = 0.05,
						set_variable = 'v',
						measured_variables = ['i','v'],
						measured_slope_tolerance_ranges = [S_mean_range,(0.9,1.1)],
						filename = 'Temporary sweep V',
						plot = True,
						plot_deviation = True,
						rawi = True,
						rawv = True,
						random_order = False,
						dwell_t = 0.04,
						channels = range(8),
						scan_keithley = True)
				
				if False and calibrate_v and calibrate_i:
				
					# Calibrate both current and voltage against test loads
					qcpass_cal &= calibrate_output_voltage_and_input_current_with_R(
							q, 
							keithley, 
							test_resistances,
							n_chs = 8,
							vstep = 0.5, 
							vmax = vmax, 
							dwell_t = 0,
							filename = 'Temporary calibration')
				
				if calibrate_v:
				
					# Calibrate voltages using multimeter
					qcpass_cal &= calibrate_output_voltage(
						q, 
						keithley, 
						vstep = 0.5, 
						vmax = vmax, 
						filename = 'Temporary calibration')
				
				if calibrate_i:
				
					# Calibrate current against test loads
					if test_resistances is not None:
						qcpass_cal &= calibrate_input_current_with_R(
							q, 
							keithley, 
							test_resistances,
							n_chs = 8,
							vstep = 0.5, 
							vmax = vmax, 
							dwell_t = 0.1,
							filename = 'Temporary current calibration')
					else:
						print("Warning: No test resistances provided; not calibrating current.")
				
				
				# Switch back to off-board 5V supply
				q.issue_command('digsup', operator='=', value=1)
					
			# Q8iv calibration routine
			if id_prefix == 'Q8iv':
		
				# Try to setup keithley
				keithley = setup_keithley()
				
				
				if high_quality_iv:
					
					# Sweep current measuring off-board voltage and on-board voltage and current, store in file
					(_,_,qc) = get_XY(
						q, 
						keithley,
						xmin = 0.0,
						xmax = 8.0, 
						xstep = 0.05,
						set_variable = 'i',
						measured_variables = ['i','v','u'],
						measured_slope_tolerance_ranges = [(0.9,1.1),R_mean_range,R_mean_range],
						filename = 'Temporary sweep I',
						plot = True,
						plot_deviation = True,
						rawi = True,
						rawv = True,
						random_order = False,
						dwell_t = 0.04,
						channels = range(8),
						scan_keithley = True)
			
					qcpass_cal &= qc
			
					# Sweep voltage measuring off-board voltage and on-board voltage and current, store in file
					(_,_,qc) = get_XY(
						q, 
						keithley,
						xmin = 0.0,
						xmax = 12.0, 
						xstep = 0.05,
						set_variable = 'v',
						measured_variables = ['i','v','u'],
						measured_slope_tolerance_ranges = [S_mean_range,(0.9,1.1),(0.9,1.1)],
						filename = 'Temporary sweep V',
						plot = True,
						plot_deviation = True,
						rawi = True,
						rawv = True,
						random_order = False,
						dwell_t = 0.04,
						channels = range(8),
						scan_keithley = True)
			
					qcpass_cal &= qc
				
				if calibrate_v:
			
					# Calibrate voltages using multimeter
					qcpass_cal &= calibrate_output_voltage(
							q, 
							keithley, 
							vstep = 0.2, 
							vmax = 12.0, 
							filename = 'Temporary calibration')
			
					# Calibrate onboard voltage measurement against (calibrated) voltage output
					calibrate_voltage(
							q,
							timeout = 4.0)
				
				if calibrate_i:
				
					# Calibrate current against test loads
					if test_resistances is not None:
						qcpass_cal &= calibrate_input_current_with_R(
							q, 
							keithley, 
							test_resistances,
							n_chs = 8,
							vstep = 0.5, 
							vmax = 12.0, 
							dwell_t = 0.1,
							filename = 'Temporary current calibration')
					else:
						print("Warning: No test resistances provided; not calibrating current.")
		
		
		########################
		### SAVE and RESTORE ###
		########################
		
		
		# Get device ID
		try:
			# Flush any faulty commands
			for i in range(3):
				q.issue_command ('id', operator='?')
			# Get ID
			id = q.issue_command ('id', operator='?', output_regex='(\w.+-[0-9A-F]+)', n_lines_requested = 1)[0][0]
		except:
			id = None
			if name:
				print ("Warning: Failed to get device ID after ID creation.")
		
		
		if id is not None:
			print ("\nSaving files...")
			safe_rename('Temporary programming log.txt', '{:} Programming Log.txt'.format(id))
			safe_rename('Temporary sweep I.csv', '{:} Sweep I.csv'.format(id))
			safe_rename('Temporary sweep I I-I.pdf', '{:} Sweep I-I.pdf'.format(id))
			safe_rename('Temporary sweep I V-I.pdf', '{:} Sweep V-I.pdf'.format(id))
			safe_rename('Temporary sweep I U-I.pdf', '{:} Sweep U-I.pdf'.format(id))
			safe_rename('Temporary sweep V.csv', '{:} Sweep V.csv'.format(id))
			safe_rename('Temporary sweep V I-V.pdf', '{:} Sweep I-V.pdf'.format(id))
			safe_rename('Temporary sweep V V-V.pdf', '{:} Sweep V-V.pdf'.format(id))
			safe_rename('Temporary sweep V U-V.pdf', '{:} Sweep U-V.pdf'.format(id))
			safe_rename('Temporary calibration.csv', '{:} Calibration Voltage.csv'.format(id))
			safe_rename('Temporary current calibration.csv', '{:} Calibration Current.csv'.format(id))
			safe_rename('Temporary calibration.csv.pdf', '{:} Calibration.pdf'.format(id))
			safe_rename('Temporary calibration.csv I-V.pdf', '{:} Sweep I-V.pdf'.format(id))
			safe_rename('Temporary calibration.csv V-V.pdf', '{:} Sweep V-V.pdf'.format(id))
		



		print ('\nFinished in {:.1f} seconds.'.format(time.time() - t_start))
		
		q.close()
		del(sys.stdout)
	
		if loop:
			response = raw_input('\nProgram another device? (y/n) : ')
		else:
			response = 'n'
	
	return (id, qcpass_id, qcpass_cal)
			

def passfail(p, bad_text = bad_text, bad_text_words = "FAIL"):
	return (good_text+'[PASS]'+normal_text if p else bad_text+'['+bad_text_words+']'+normal_text)
	

def batch_programme_and_log(
			slots_to_program, customer_or_quote = '', 
			filename_prefix = '~/Qontrol Dropbox/Engineering/Outgoing devices/', 
			flash_script_filename = '~/Qontrol Dropbox/Code/Firmware/qontrol_firmware.X/flash.sh', 
			config_filename = '~/Qontrol Dropbox/Code/Firmware/qontrol_firmware.X/src/config.h', 
			record_filename = 'Programmed devices.csv', 
			force_preserve_nvm = False,
			calibrate_v = True,
			calibrate_i = True,
			high_quality_iv = True,
			name = True,
			device_type = 'q8iv',
			force_names = True,
			hang = True,
			cut_power_at_stop = True,
			test_resistance_value = 1000,
			target_serial_port_glob = '/dev/tty.usbserial-FT2MCR74',
			control_serial_port_glob = '/dev/tty.usbserial-FT2MCM5U',
			failure_repeats = 3,
			psu_voltage = [15.0, 15.0, 5.0],
			psu_current = [1.5, 1.0, 1.0],
			flash_it = True):
	
	# Aspirations for this routine:
	# * Copy source code to temporary directory at start
	# * Be able to modify config.h
	# * Be able to sense which slots are occupied
	# * Report who the database is being populated for at start
	
	# Time ourselves
	t_start = time.time()
	
	# Go to output directory
	os.chdir(os.path.expanduser(filename_prefix))
	
	
	# Get fresh IDs to be applied to these devices
	ids = []
	
	for e,i in enumerate(slots_to_program):
		if name:
			if isinstance(force_names, list):
				print ("  Forcing name '{:}' in slot {:}".format(force_names[e], i))
				new_device_id = force_names[e]
			elif isinstance(force_names, bool) and force_names == True:
				(new_device_id, _, _) = get_new_device_id(
										id_prefix, 
										None, 
										record_filename, 
										preexisting_ids = ids)
			else:
				new_device_id = None
			
			ids.append(new_device_id)
		else:
			ids.append(None)
	
	
	
	print ("\nProgramming boards in:")
	for board_i,board in enumerate(slots_to_program):
		id = ids[board_i] if ids[board_i] != None else 'unspecified'
		print ("  slot {:} with ID {:}{:}{:}".format(board, good_text, id, normal_text))
	
	
	# Setup PSU
	psu = setup_psu()
	psu.set_enable(False)
	voltages = psu_voltage
	currents = psu_current
	for i in range(3):
		ch = i+1
		psu.set_limit(voltages[i], ch, "voltage")
		psu.set_limit(currents[i], ch, "current")
	psu.set_enable(True)
	nap(3.0)
	print("\nInitial DC power state:")
	for i in range(3):
		ch = i+1
		vm = psu.measure(ch, "voltage")
		vg = (good_text if abs(vm-voltages[i])<0.2 else bad_text)+str(vm)+normal_text
		im = psu.measure(ch, "current")
		print("  Output {:} is {:} V @ {:} A".format(ch, vg, im ) )
	
	psu.set_remote(False)
	
	
	# Find control serial ports
	serial_ports = glob.glob(control_serial_port_glob)
	if len(serial_ports) > 1:
		raise RuntimeError('Found multiple potential serial ports for control qontroller. Specify the correct one manually.')
	
	serial_port_name = serial_ports[0]
	
	# Setup control qontroller
	qc = setup_qontroller(
			serial_port_name = serial_port_name,
			timeout = 0.1,
			n_chs = 16)
	
	# Stretch our legs
	ctrl_boards = range(12)
	ctrl_leds = [12,13]
	ctrl_buzzer = 14
	ctrl_power = 15
	
	for ch in slots_to_program:
		qc.v[ch]=12
		nap(0.01)
		qc.v[ch]=0
	
# 	qc.v[ctrl_power] = 12
# 	nap(0.5)
# 	qc.v[ctrl_power] = 0
	
	qc.v[ctrl_buzzer] = 2
	qc.v[ctrl_leds[0]] = 5
	nap(0.2)
	qc.v[ctrl_leds[1]] = 5
	nap(0.2)
	qc.v[ctrl_leds[0]] = 0
	nap(0.2)
	qc.v[ctrl_leds[1]] = 0
	qc.v[ctrl_buzzer] = 0
	
	
	
		
	# Get load resistances
	
	try:
		keithley = setup_keithley()
		keithley.set_integration_time(10.0)
		keithley.set_measurement ('R')
		resistances = []
		sys.stdout.write("\r  Measuring test resistances:\n")
		for ch in range(8):
			keithley.set_scanner_channel(ch+1)
			resistances.append(keithley.measure())
			if abs(resistances[-1] - test_resistance_value)/test_resistance_value < 0.2:
				style_text = good_text
			else:
				style_text = bad_text
			
			sys.stdout.write("    Channel {:}:  {:}{:.3f}{:} Ohm\n".format(ch, style_text, resistances[ch-1], normal_text))
		
		keithley.set_integration_time(1.0)
		
		del(keithley)
	
		for ch in range(8):
			if abs(resistances[ch] - test_resistance_value)/test_resistance_value > 0.2:
				raise RuntimeError("Resistance on channel {:} was out of legal range ({:} Ohm, target {:} Ohm".format(ch, resistances[ch], test_resistance_value))
	
	except Exception as e:
		resistances = None
		sys.stdout.write ("  Failed to measure test resistances (error {:}).\n".format(e))
	
	# Function for generating beeps
	def beep_pattern(pattern=[0.5,0.5], led=1):
		for i,p in enumerate(pattern):
			try:
				t = p[0]
				v = p[1]
			except:
				t = p
				v = 1
			if i%2:
				qc.v[ctrl_leds[led]] = 0
				qc.v[ctrl_buzzer] = 0
				nap(t)
			else:
				qc.v[ctrl_buzzer] = v
				qc.v[ctrl_leds[led]] = 5
				nap(t)
		
		qc.v[ctrl_leds[led]] = 0
		qc.v[ctrl_buzzer] = 0
	
	
	# Keep track of pass/fail of each board
	qcpass_boards = []
	
	# Keep track of progress
	t_init = time.time()
	boards_done = 0
	boards_tot = len(slots_to_program)
	
	# Loop through devices to program, and program them!
	for board_i,board in enumerate(slots_to_program):
		
		# If we fail, try again three times
		for _ in range(3):
		
			if len(slots_to_program) > 1:
				sys.stdout.write("\n")
				for _ in range(50):
					sys.stdout.write("-")
				print ("\nProgramming board in slot {:}...".format(board))
				for _ in range(50):
					sys.stdout.write("-")
				sys.stdout.write("\n\n")
				sys.stdout.flush()
			else:
				print ("\nProgramming board in slot {:}...\n".format(board))
		
		
			# Deactivate other boards
			for ch in ctrl_boards:
				qc.v[ch] = 0
		
			# Ensure power is on
			qc.v[ctrl_power] = 0
		
			# Connect board
			qc.v[ctrl_boards[board]] = 12
		
		
			# Proceed with programming
# 			try:
			(id, qcpass_id, qcpass_cal) = programme_and_log(
					record_filename = record_filename,
					customer_or_quote = customer_or_quote,
					force_preserve_nvm = force_preserve_nvm,
					force_id = ids[board_i],
					calibrate_v = calibrate_v,
					calibrate_i = calibrate_i,
					high_quality_iv = high_quality_iv,
					name = name,
					device_type = device_type,
					test_resistances = resistances,
					qontroller_serial_port_glob = target_serial_port_glob,
					flash_script_filename = flash_script_filename,
					config_filename = config_filename,
					flash_it = flash_it,
					loop = False,
					psu = psu)
# 			except RuntimeError as e:
# 				print ("  Board in slot {:} failed to programme.\n   Error was '{:}'.".format(board, e) )
# 				(qcpass_id, qcpass_cal) = (False, False)
		
			qcpass_boards.append((qcpass_id, qcpass_cal))
			ids[board_i] = id
			
		
			# Disconnect board
			qc.v[ctrl_boards[board]] = 0
			
			
			if board != slots_to_program[-1]:
				if all((qcpass_id, qcpass_cal)):
					# Make a happy noise
					beep_pattern([(0.1,1.2), 0.05, (0.05,1.2), 0.02, (0.03,1.2), 0.02, (0.4,1.6)])
				else:
					# Make a sad noise
					beep_pattern([(0.05,1.2), 0.05, (0.05,1.2), 0.05, (0.05,1.2), 0.15, (0.2,1.6), 0.05, (0.2,1.6), 0.05, (0.05,1.2), 0.05, (0.05,1.2), 0.05, (0.05,1.2)], led=0)
			else:
				# Make a happy noise, because we're done!
					beep_pattern([(0.1,5), 0.05, (0.05,5), 0.02, (0.03,5), 0.02, (0.08,4), 0.05, (0.25,4), 0.08,(0.6,7)])
			
			break
		
		# Update user on our progress
		boards_done += 1
		t_now = time.time()
		if boards_done < boards_tot:
			print ("About {:.1f} minutes remaining.".format( (t_now-t_init)/boards_done * (boards_tot-boards_done)/60 ) )
	
	# Do final checks on whole chain
	print ("\nFinal daisy chain configuration:")
	
	# Deactivate other boards
	for ch in ctrl_boards:
		qc.v[ch] = 0
	
	# Turn power off
	# qc.v[ctrl_power] = 12
	
	# Connect to first board
	qc.v[ctrl_boards[0]] = 12
	
	# Turn power on
	qc.v[ctrl_power] = 0
	
	# Wait for startup
	# qc.wait(3)
	
	try:
		q = setup_qontroller(
				serial_port_name = target_serial_port_glob,
				timeout = 0.2)
				
		response = q.issue_command('nup',None,'=',0)
	
		response = q.issue_command('nupall',None,'?')
	
		qcpass_chain = (len(response) == len(slots_to_program))
	
		for line in response:
			print ('  ' + line[0])
	except RuntimeError:
		qcpass_chain = False
	
	# Disconnect
	qc.v[ctrl_boards[0]] = 0
	
	# Produce report
	qcpass_labels = ["  ID  ", " Cal  "]
	qcpass_boards_all = [all(e) for e in qcpass_boards]
	qcpass_boards_text = [' '.join([passfail(p) for p in qcpass]) for qcpass in qcpass_boards]
	print ("\nBatch programming succeeded {:.2f}%".format(100.0*qcpass_boards_all.count(True)/len(qcpass_boards_all)))
	
	id_str = lambda x: '****' if x is None else str(x)
	
	try:
		s = "  Board {i:2}, {ID: <11}: ".format(i=board, ID=id_str(ids[0]))
	except Exception as e:
		from IPython import embed
		embed()
		raise e
		
	l = len(s)
	print (" " * l + " ".join(qcpass_labels))
	for board_i,board in enumerate(slots_to_program):
		print ("  Board {i:2}, {ID: >11}: {qc}".format(i=board, ID=id_str(ids[board_i]), qc=qcpass_boards_text[board_i]) )
	print ("  Daisy chain config:    {:}".format(passfail(qcpass_chain)) )
	
	
	
	print("\nFinal DC power state:")
	for i in range(3):
		ch = i+1
		vm = psu.measure(ch, "voltage")
		vg = (good_text if abs(vm-voltages[i])<0.2 else bad_text)+str(vm)+normal_text
		im = psu.measure(ch, "current")
		print("  Output {:} is {:} V @ {:} A".format(ch, vg, im ) )
	
	# Turn power off
	if cut_power_at_stop:
		qc.v[ctrl_power] = 12
		qc.v[ctrl_power] = 5
		
		try:
			psu.set_enable(False)
			psu.set_remote(False)
		except:
			print("Failed to disable PSU.")
	
	if hang:
		
		# Loop the 'done' noise forever
		sys.stdout.write("\nComplete! Remove boards, press Ctrl-C to exit...")
		sys.stdout.flush()
		try:
			nap(10)
			while True:
				beep_pattern([0.5, 0.2, 0.05, 0.05, 0.05, 0.05])
			
				nap(30)
		except KeyboardInterrupt:
			for i in range(16):
				if i != ctrl_power: # Keep power off
					qc.v[i] = 0
					
					
def batch_check(
			slots_to_check,
			filename_prefix = '~/Qontrol Dropbox/Engineering/Outgoing devices/', 
			sweep_v = True,
			sweep_i = True,
			high_quality_iv = False,
			test_resistance_value = 1000,
			target_serial_port_glob = '/dev/tty.usbserial-FT2MCR74',
			control_serial_port_glob = '/dev/tty.usbserial-FT2MCM5U',
			failure_repeats = 3):
	
	
	# Time ourselves
	t_start = time.time()
	
	# Go to output directory
	os.chdir(os.path.expanduser(filename_prefix))
	
	
	# Tee output to copy stdout to file
	sys.stdout = Tee('Temporary check file.txt', 'w')
	
	
	datestamp = datetime.datetime.now().strftime("%Y-%m-%d %Hh%Mm%S")
	print ("\nTime is {:}".format(datestamp))
	
	print ("\nChecking boards in:")
	for board_i,board in enumerate(slots_to_check):
		print ("  slot {:}".format(board))
	
	
	
	# Find control serial ports
	serial_ports = glob.glob(control_serial_port_glob)
	if len(serial_ports) > 1:
		raise RuntimeError('Found multiple potential serial ports for control qontroller. Specify the correct one manually.')
	
	serial_port_name = serial_ports[0]
	
		
	# Find target serial port
	serial_ports = glob.glob(target_serial_port_glob)
	if len(serial_ports) > 1:
		raise RuntimeError('Found multiple potential serial ports for control qontroller. Specify the correct one manually.')
	
	target_serial_port_name = serial_ports[0]
	
	
	# Setup control qontroller
	qc = setup_qontroller(
			serial_port_name = serial_port_name,
			timeout = 0.1,
			n_chs = 16)
	
	# Stretch our legs
	ctrl_boards = range(12)
	ctrl_leds = [12,13]
	ctrl_buzzer = 14
	ctrl_power = 15
	
	for ch in slots_to_check:
		qc.v[ch]=12
		nap(0.01)
		qc.v[ch]=0
	
	qc.v[ctrl_buzzer] = 2
	qc.v[ctrl_leds[0]] = 5
	nap(0.2)
	qc.v[ctrl_leds[1]] = 5
	nap(0.2)
	qc.v[ctrl_leds[0]] = 0
	nap(0.2)
	qc.v[ctrl_leds[1]] = 0
	qc.v[ctrl_buzzer] = 0
	
	
	
		
	# Get load resistances
	
	try:
		keithley = setup_keithley()
		keithley.set_integration_time(10.0)
		keithley.set_measurement ('R')
		resistances = []
		sys.stdout.write("\r  Measuring test resistances:\n")
		for ch in range(8):
			keithley.set_scanner_channel(ch+1)
			resistances.append(keithley.measure())
			if abs(resistances[-1] - test_resistance_value)/test_resistance_value < 0.2:
				style_text = good_text
			else:
				style_text = bad_text
			
			sys.stdout.write("    Channel {:}:  {:}{:.3f}{:} Ohm\n".format(ch, style_text, resistances[-1], normal_text))
			sys.stdout.flush()
		
		resistance_mean = numpy.mean(resistances)/1000.0 # kOhm
		
		keithley.set_integration_time(1.0)
		del(keithley)
	
	except Exception as e:
		resistances = None
		sys.stdout.write ("  Failed to measure test resistances (error {:}).\n".format(e))
		raise RuntimeError("Failed to measure resistances. Keithley connected?")
	
	keithley = setup_keithley()
	
	for ch in range(8):
		if abs(resistances[ch] - test_resistance_value)/test_resistance_value > 0.2:
			raise RuntimeError("Resistance on channel {:} was out of legal range ({:} Ohm, target {:} Ohm".format(ch, resistances[ch], test_resistance_value))
	
	# Function for generating beeps
	def beep_pattern(pattern=[0.5,0.5], led=1):
		for i,p in enumerate(pattern):
			try:
				t = p[0]
				v = p[1]
			except:
				t = p
				v = 1
			if i%2:
				qc.v[ctrl_leds[led]] = 0
				qc.v[ctrl_buzzer] = 0
				nap(t)
			else:
				qc.v[ctrl_buzzer] = v
				qc.v[ctrl_leds[led]] = 5
				nap(t)
		
		qc.v[ctrl_leds[led]] = 0
		qc.v[ctrl_buzzer] = 0
	
	
	# Keep track of pass/fail of each board
	qcpass_boards = []
	ids = []
	
	# Keep track of progress
	t_init = time.time()
	boards_done = 0
	boards_tot = len(slots_to_check)
	
	# Loop through devices and test
	for board_i,board in enumerate(slots_to_check):
		
		
		if len(slots_to_check) > 1:
			sys.stdout.write("\n")
			for _ in range(50):
				sys.stdout.write("-")
			print ("\nTesting board in slot {:}...".format(board))
			for _ in range(50):
				sys.stdout.write("-")
			sys.stdout.write("\n\n")
			sys.stdout.flush()
		else:
			print ("\nTesting board in slot {:}...\n".format(board))
	
	
		# Deactivate other boards
		for ch in ctrl_boards:
			qc.v[ch] = 0
	
		# Ensure power is on
		qc.v[ctrl_power] = 0
	
		# Connect board
		qc.v[ctrl_boards[board]] = 12
		
		time.sleep(2.0)
		
		
		# Setup target qontroller
		q = setup_qontroller(
				serial_port_name = target_serial_port_name,
				timeout = 0.1,
				n_chs = 8)
		
		id = q.device_id
		ids.append(id)
		
		# Do device-specific setup
		ob = re.match('(\w+)-([0-9a-fA-F\*]+)', id)
		got_device_type = ob is not None
		
		if got_device_type:
			dev_type,dev_num = ob.groups()
			
			print ("\nPerforming device-type-specific setup...")
			print ("  Device type is '{:}'".format(dev_type))
		
			if dev_type == 'Q8b':
				# Use the on-board 5V regulator for calibration
				q.issue_command('digsup', operator='=', value=0)
				print ("  Using on-board 5V regulator for check...")
		else:
			dev_type = ''
			print ("\nWarning: Unable to detect device type of device with ID '{:}'.".format(id))
	
	
		# Proceed with checking
		qcpass_v = qcpass_i = True
		try:
			if sweep_v:
				print ("\nSweeping voltage...\n")
				
				vmax = float(q.vfull[:-1])
				vstep = 0.05 if high_quality_iv else 1.0
				
				(_,_,qcpass_v) = get_XY(
					q, 
					keithley,
					xmin = 0.0,
					xmax = vmax, 
					xstep = vstep, 
					set_variable = 'v',
					measured_variables = ['v','u','i'],
					measured_slope_tolerance_ranges = [(0.99,1.01), (0.99,1.01), (0.9/resistance_mean,1.1/resistance_mean)],
					measured_offset_tolerance_ranges = [(-0.1,0.1), (-0.1,0.1), (-0.5,0.5)],
					filename = 'Temporary sweep', 
					plot = True,
					plot_deviation = False,
					rawi = False,
					rawv = False,
					random_order = True,
					print_uncertainties = True,
					failure_repeats = 2,
					dwell_t = 0.00,
					channels = range(8),
					scan_keithley = True)
					
			if sweep_i:
				print ("\nSweeping current...\n")
				
				imax = float(q.ifull[:-2])
				istep = 0.05 if high_quality_iv else 0.5
				
				(_,_,qcpass_i) = get_XY(
					q, 
					keithley,
					xmin = 0.0,
					xmax = imax, 
					xstep = istep, 
					set_variable = 'i',
					measured_variables = ['v','u','i'],
					measured_slope_tolerance_ranges = [(0.99,1.01), (0.99,1.01), (0.9,1.1)],
					measured_offset_tolerance_ranges = [(-0.1,0.1), (-0.1,0.1), (-0.5,0.5)],
					filename = 'Temporary sweep', 
					plot = True,
					rawi = False,
					rawv = False,
					random_order = False,
					print_uncertainties = True,
					failure_repeats = 2,
					dwell_t = 0.05,
					channels = range(8),
					scan_keithley = True)
			
		except RuntimeError as e:
			print ("  Board in slot {:} failed to check.\n   Error was '{:}'.".format(board, e) )
			(qcpass_v, qcpass_i) = (False, False)
	
		qcpass_boards.append((qcpass_v, qcpass_i))
		
		
		safe_rename('Temporary sweep V-V.pdf', '{:} Sweep V-V check.pdf'.format(id))
		safe_rename('Temporary sweep I-V.pdf', '{:} Sweep I-V check.pdf'.format(id))
		safe_rename('Temporary sweep U-V.pdf', '{:} Sweep U-V check.pdf'.format(id))
		safe_rename('Temporary check file.txt', '{:} check.txt'.format(id))
		
		# Reset device setup
		if dev_type == 'Q8b':
			q.issue_command('digsup', operator='=', value=1)
		
		del(q)
		
	
		# Disconnect board
		qc.v[ctrl_boards[board]] = 0
		
		
		if board != slots_to_check[-1]:
			if all((qcpass_v, qcpass_i)):
				# Make a happy noise
				beep_pattern([(0.1,1.2), 0.05, (0.05,1.2), 0.02, (0.03,1.2), 0.02, (0.4,1.6)])
			else:
				# Make a sad noise
				beep_pattern([(0.05,1.2), 0.05, (0.05,1.2), 0.05, (0.05,1.2), 0.15, (0.2,1.6), 0.05, (0.2,1.6), 0.05, (0.05,1.2), 0.05, (0.05,1.2), 0.05, (0.05,1.2)], led=0)
		else:
			# Make a happy noise, because we're done!
				beep_pattern([(0.1,5), 0.05, (0.05,5), 0.02, (0.03,5), 0.02, (0.08,4), 0.05, (0.25,4), 0.08,(0.6,7)])
		
		# Update user on our progress
		boards_done += 1
		t_now = time.time()
		if boards_done < boards_tot:
			print ("About {:.1f} minutes remaining.".format( (t_now-t_init)/boards_done * (boards_tot-boards_done)/60 ) )
	
	# Do final checks on whole chain
	print ("\nFinal daisy chain configuration:")
	
	# Deactivate other boards
	for ch in ctrl_boards:
		qc.v[ch] = 0
	
	# Turn power off
	# qc.v[ctrl_power] = 12
	
	# Connect to first board
	qc.v[slots_to_check[0]] = 12
	
	# Turn power on
	qc.v[ctrl_power] = 0
	
	# Wait for startup
	# qc.wait(3)
	
	try:
		q = setup_qontroller(
				serial_port_name = target_serial_port_glob,
				timeout = 0.2)
				
		response = q.issue_command('nup',None,'=',0)
	
		response = q.issue_command('nupall',None,'?')
	
		qcpass_chain = (len(response) == len(slots_to_check))
	
		for line in response:
			print ('  ' + line[0])
	except RuntimeError:
		qcpass_chain = False
	
	# Disconnect
	qc.v[ctrl_boards[0]] = 0
	
	# Produce report
	qcpass_labels = ["  I   ", "  V   "]
	qcpass_boards_all = [all(e) for e in qcpass_boards]
	qcpass_boards_text = [' '.join([passfail(p) for p in qcpass]) for qcpass in qcpass_boards]
	print ("\nBatch testing succeeded {:.2f}%".format(100.0*qcpass_boards_all.count(True)/len(qcpass_boards_all)))
	l = len("  Board {i:2}, {id: <9}: ".format(i=board, id=ids[0]))
	print (" " * l + " ".join(qcpass_labels))
	for board_i,board in enumerate(slots_to_check):
		print ("  Board {i:2}, {id: >9}: {qc}".format(i=board, id=ids[board_i], qc=qcpass_boards_text[board_i]) )
	print ("  Daisy chain config:  {:}".format(passfail(qcpass_chain, warn_text, 'WARN')) )
	
	print ("  Check took {:.1f} s".format(time.time() - t_start))
	
	# Turn power off
	qc.v[ctrl_power] = 12
	qc.v[ctrl_power] = 9
	
	# Delete tee
	del(sys.stdout)
	


def test_cable (
			cable_type = "CAB12", 
			note = None,
			filename_prefix = '~/Qontrol Dropbox/Engineering/Outgoing devices/', 
			control_serial_port_glob = '/dev/tty.usbserial-FT2MCM5U',
			stdev_thresh = 5 # Num of std deviations for resistance to [PASS]
			):
	
	
	# Go to output directory
	os.chdir(os.path.expanduser(filename_prefix))
	
	# Check cable, set properties
	cable_type = cable_type.upper()
	if cable_type == "CAB12":
		npins = 96
		cable_length = 1.0
	elif cable_type == "CAB8":
		npins = 64
		cable_length = 1.0
	else:
		raise AttributeError("Specified cable_type '{:}' is not recognised.".format(cable_type) )
	
	dmm = setup_keithley(serial_port_glob = '/dev/tty.usbserial-A*')
	
		
	# Copy stdout to log file
	sys.stdout = Tee('{:} {:}.txt'.format(cable_type, time.strftime("%Y-%m-%d %H-%M-%S") ), 'w')
		
	# Preamble
	print ("\nTesting continuity of cable:\n  {:}\n  {:} conductors\n  {:}".format(cable_type, npins, time.strftime("%Y-%m-%d %H:%M:%S") ) )
	if note is not None:
		print ("  note: {:}".format(note) )
	
	# Configure for resistance measurements
	dmm.set_measurement ('resistance')
	
	# Find control serial ports
	serial_ports = glob.glob(control_serial_port_glob)
	if len(serial_ports) > 1:
		raise RuntimeError('Found multiple potential serial ports for control qontroller. Specify the correct one manually.')
	
	serial_port_name = serial_ports[0]
	
	# Setup control qontroller
	qc = setup_qontroller(
			serial_port_name = serial_port_name,
			timeout = 0.1,
			n_chs = 16)
	
	# Stretch our legs
	ctrl_boards = range(12)
	ctrl_leds = [12,13]
	ctrl_buzzer = 14
	ctrl_power = 15
	
	# Function for generating beeps
	def beep_pattern(pattern=[0.5,0.5], led=1):
		for i,p in enumerate(pattern):
			try:
				t = p[0]
				v = p[1]
			except:
				t = p
				v = 1
			if i%2:
				qc.v[ctrl_leds[led]] = 0
				qc.v[ctrl_buzzer] = 0
				nap(t)
			else:
				qc.v[ctrl_buzzer] = v
				qc.v[ctrl_leds[led]] = 5
				nap(t)
		
		qc.v[ctrl_leds[led]] = 0
		qc.v[ctrl_buzzer] = 0
	
	
	# Keep track of pass/fail of each board
	qcpass_boards = []
	
	# Get existing shunt resistances
	
	# Disconnect all channels
	qc.v[:] = 0
	
	# Scan through and collect shunt resistances
	# Due to a mis-wiring on TP12 HW00, we can't disconnect loads from cable
	# shunts = []
	# print ("\nMeasuring existing shunt resistances...")
	# for dmm_ch in range(8):
	#	dmm.set_scanner_channel (dmm_ch+1)
	#	shunts.append(dmm.measure())
	#	print ("  Channel {:}: {:} Ohms".format(dmm_ch, shunts[-1]) )
	
	
	# Loop through devices to program, and program them!
	print ("\nScanning conductors..." )
	for slot in range(npins/8):
		
		# Check we're on a valid slot
		if slot not in ctrl_boards:
			raise RuntimeError("Too many pins specified for cable.")
		
		# Activate slot
		qc.v[slot] = 12
		
		
		for dmm_ch in range(8):
			
			qc.v[ctrl_leds[1]] = 5.0*(dmm_ch % 2)
			
			dmm.set_scanner_channel (dmm_ch+1)
			R = dmm.measure()
 			# R_shunt = shunts[dmm_ch]
			shunt_resistance = 0 #abs((R*R_shunt)/(R_shunt-R))
			qcpass_boards.append((R, shunt_resistance))
			
			sys.stdout.write_to_screen_only("  Conductor {: >2}/{:} ({:.3f} Ohm)\r".format(slot*8+dmm_ch+1,npins, R))
			sys.stdout.flush()
		
		# Deactivate slot
		qc.v[slot] = 0
	
	sys.stdout.write("\n")
	
	# Deactivate all
	qc.v[:] = 0
	
	
	# Produce report
	Rs = [elem[0] for elem in qcpass_boards]
	R_mean = numpy.mean(Rs)
	R_stdev = numpy.std(Rs)
	qcpasses = []
	print ("\nConductor test results:")
	for i,elem in enumerate(qcpass_boards):
		R = elem[0]
		pf = abs(R - R_mean) < stdev_thresh*R_stdev
		qcpasses.append(pf)
		print ("  Pin {:02}: {:} {:.3f} Ohm".format(i+1, passfail(pf), R) )
	
	print ("\nCable return resistance:\n  mean {:.3f} +/- {:.3f} Ohm\n  min {:.3f} Ohm, max {:.3f} Ohm".format(R_mean, R_stdev, min(Rs), max(Rs) ) )
	print ("\nCable overall result: {:}".format(passfail(all(qcpasses))))
	
	if all(qcpasses):
		# Make a happy noise
		beep_pattern([(0.1,5), 0.05, (0.05,5), 0.02, (0.03,5), 0.02, (0.08,4), 0.05, (0.25,4), 0.08,(0.6,7)])
	else:
		# Make a sad noise
		beep_pattern([(0.05,1.2), 0.05, (0.05,1.2), 0.05, (0.05,1.2), 0.15, (0.2,1.6), 0.05, (0.2,1.6), 0.05, (0.05,1.2), 0.05, (0.05,1.2), 0.05, (0.05,1.2)], led=0)

	# Stop recording to file
	del(sys.stdout)
	
	# Loop the 'done' noise forever
	print ("\nComplete! Please remove cable.")


def get_documentation_data(measurement, hw_rev = "05"):
	
	def qc_setup():
		return setup_qontroller(serial_port_name = '/dev/tty.usbserial-FT2MCM5U', n_chs = 16)
		
	def q_setup(n_chs = 8, catch_errs = True):
		q = setup_qontroller(serial_port_name = '/dev/tty.usbserial-FT2MCR7R', n_chs = n_chs, catch_errs = catch_errs)
		global device_type
		device_type = q.chain[0]['device_type']
		return q
	
	def ke_setup():
		return setup_keithley(serial_port_glob = '/dev/tty.usbserial-A*')
	
	datestamp = datetime.datetime.now().strftime("%Y-%m-%d %Hh%Mm%S")
	
	def make_filename(
			payload = "Documentation data", 
			extension = ".csv", 
			target_directory = "~/Qontrol Dropbox/Engineering/Q8iv/Characterisation"):
			
		return os.path.expanduser(target_directory + "/" + " ".join([device_type,"HW"+hw_rev, payload, datestamp]) + extension)
	
	
	# Copy stdout to log file
	sys.stdout = Tee('Documentation data log.txt', 'w')
	
	if measurement == "transients":
		board = 0
		chan = 7
		vtargs = [12]

		qc = qc_setup()

		qc.v[:] = 0
		qc.v[:] = 0
		qc.v[board] = 12

		q = q_setup()
		
		time.sleep(1.0)
		print("Start triggering...")
		time.sleep(5.0)
		n = 10000
		for i in range(n):
			if i%10 == 0:
				print ("{:}% complete".format(100.*i/n))
			q.v[chan] = vtargs[i%len(vtargs)]
			time.sleep(0.05)
			q.v[chan] = 0
	
	if measurement == "speed":
		board = 0
		chan = 7

		qc = qc_setup()

		qc.v[:] = 0
		qc.v[:] = 0
		qc.v[board] = 12

		q = q_setup()
		
		
		def get_val(i):
			return bytearray([int(i/256),int(i)-int(i/256)*256])
		
		def get_v_cmd(i):
			c = bytearray(b'\x81\x00\x00\x00')
			c.append(chan)
			c.extend(get_val(i))
			return c
		
		v_up_cmd = get_v_cmd(0xFFFF)
		v_dn_cmd = get_v_cmd(0x0000)
		
		print("Start triggering...")
		n = 100000
		
		
		print(dir(q.serial_port))
		try:
			for i in range(n):
				if i%100 == 0:
					print ("{:}% complete".format(100.*i/n))
			
				# Alternate up and down voltages
				if i%2:
					q.serial_port.write(v_up_cmd)
				else:
					q.serial_port.write(v_dn_cmd)
			
				q.serial_port.flush()
		except:
			print ("Keyboard abort")
			pass
		
		q.serial_port.reset_input_buffer()
	
	if measurement == "vpower" or measurement == "ipower":
		
		board = 0
		chan = 7
		keithley_nplcs = 10.0
		
		q = q_setup()
		q.v[:] = 0
		qc = qc_setup()
		qc.v[:] = 0
		
		ke = ke_setup()
		ke.set_integration_time(keithley_nplcs)
		ke.set_scanner_channel(chan+1)
		
		# Get test resistance value
		time.sleep(0.1)
		ke.set_measurement ('R')
		R = ke.measure()
		time.sleep(0.1)
		
		# Prepare to sweep
		ke.set_measurement('V')
		qc.v[board] = 12
		
		vmin,vmax = 0,q.v_full
		imin,imax = 0,q.i_full
		
		param = "V" if measurement == 'vpower' else "I"
		
		print ("\nSweeping V from {:} to {:}; I from {:} to {:}".format(vmin,vmax,imin,imax))
		
		# Sweep voltage over full range
		nsteps = 80
		UJs,Vs,Is = [],[],[] # Nom V or I, Keithley V, onboard I
		for n in range(nsteps):
			if measurement == "vpower":
				UJ = n*(vmax-vmin)/(nsteps-1)
				q.v[chan] = UJ
			elif measurement == "ipower":
				UJ = n*(imax-imin)/(nsteps-1)
				q.i[chan] = UJ
			
			time.sleep(0.05)
			ke.measure()
			ke.measure()
			V = ke.measure()
			I = q.i[chan]
			
			print ("For V|Inom = {:}, Vact = {:}, I = {:}".format(UJ,V,I))
			
			UJs.append(UJ)
			Vs.append(V)
			Is.append(I)
		
		Ps = [Vs[i]*Is[i] for i in range(nsteps)]
		
		
		
		filename = make_filename("Power "+measurement)
		
		print ("\nSaving to file '{:}'".format(filename))
		f = open(os.path.expanduser(filename), "w+")
		
		
		f.write("{:},{:}\n".format(
									"Time",
									str(datetime.datetime.now())))
		f.write("{:},{:}\n".format(
									"Device ID",
									q.device_id))
		f.write("{:},{:}\n".format(
									"Device channel",
									chan))
		f.write("{:},{:}\n".format(
									"Keithley number of integrated powerline cycles",
									keithley_nplcs))
		f.write("{:},{:}\n".format(
									"Load resistance (Ohm)",
									R))
									
									
									
									
									
									
		f.write("{:},{:},{:},{:}\n".format(
									"Nominal "+param,
									"Keithley voltage (V)",
									"Onboard current (mA)",
									"Power (mW)"))
		
		for j in range(len(UJs)):
			f.write("{:},{:.6f},{:.6f},{:.6f}\n".format(UJs[j],Vs[j],Is[j],Ps[j]))
		
		f.close()
	
	if measurement == "smallvsig" or measurement == "smallisig":
		
		board = 0
		chan = 0
		keithley_nplcs = 10.0
		
		imin = 0x0000
		imax = 0xFFFF
		
		# Script
		
		qc = qc_setup()
		qc.v[:] = 0
		
		ke = ke_setup()
		ke.set_scanner_channel(chan+1)
		ke.set_integration_time(keithley_nplcs)
		ke.set_measurement ('R')
		R = ke.measure()
		ke.set_measurement ('V')
		
		qc.v[board] = 12
		
		q = q_setup()
		q.v[:] = 0
		
		
		cmd = bytearray() #81 00 0000 01 4000
		cmd.append(0x81) # Single channel write
		cmd.append(0x00) # Voltage
		cmd.append(0x00)
		cmd.append(0x00)
		cmd.append(chan) # Channel address
		
		val = bytearray()
		val = b'\x00\x00' # Value
		
		def get_val(i):
			return bytearray([int(i/256),int(i)-int(i/256)*256])
		
		def get_v_cmd(i):
			c = bytearray(b'\x81\x00\x00\x00')
			c.append(chan)
			c.extend(get_val(i))
			return c
		
		def get_i_cmd(i):
			c = bytearray(b'\x81\x01\x00\x00')
			c.append(chan)
			c.extend(get_val(i))
			return c
		
		results = []
		
		q.serial_port.write(get_v_cmd(0))
		
		print ("\nStepping...")
		t_start = time.time()
		for i in range(imin,imax):
			if i%64 == 0 and i != 0:
				t_per = (time.time() - t_start)/i
				t_left = t_per * (imax-i)
				print ("{: 5}/{:} ({:.1f} s to go)".format(i,imax,t_left))
				
			results.append({})
			
			if measurement == "smallvsig":
				q.serial_port.write(get_v_cmd(i))
			elif measurement == "smallisig":
				q.serial_port.write(get_i_cmd(i))
			time.sleep(0.050)
			
			results[-1].update({'i':i})
			
			try:
				results[-1].update({'uafter':q.v[chan]})
			except:
				results[-1].update({'uafter':-1})
			try:
				results[-1].update({'iafter':q.i[chan]})
			except:
				results[-1].update({'iafter':-1})
			try:
				results[-1].update({'vafter':ke.measure()})
			except:
				results[-1].update({'vafter':-1})
		
		
		# Write to file
		if measurement == "smallvsig":
			step_type = "voltage"
		elif measurement == "smallisig":
			step_type = "current"
		filename = make_filename("Small "+step_type+" steps")
		
		print ("\nSaving to file '{:}'".format(filename))
		f = open(os.path.expanduser(filename), "w+")
		
		
			
		
		f.write("{:},{:}\n".format(
									"Time",
									str(datetime.datetime.now())))
		f.write("{:},{:}\n".format(
									"Device ID",
									q.device_id))
		f.write("{:},{:}\n".format(
									"Device channel",
									chan))
		f.write("{:},{:}\n".format(
									"Keithley number of integrated powerline cycles",
									keithley_nplcs))
		f.write("{:},{:}\n".format(
									"Load resistance (Ohm)",
									R))
									
									
									
									
									
									
		f.write("{:},{:},{:},{:}\n".format(
									"Integer "+step_type,
									"Keithley voltage (V)",
									"Onboard voltage (V)",
									"Onboard current (mA)"))
		
		for j in range(len(results)):
			i = results[j]['i']
			dv = results[j]['vafter']
			du = results[j]['uafter']
			di = results[j]['iafter']
			f.write("{:},{:.6f},{:.6f},{:.6f}\n".format(i,dv,du,di))
		
		f.close()
		
	if measurement == "precision":
		from numpy import std as std_dev
		from numpy import mean as mean
		
		# Parameters
		
		board = 0
		chan = 0
		n_measurements = 100
		keithley_nplcs = 10.0
		
		# Script
		
		qc = qc_setup()
		qc.v[:] = 0
		
		ke = ke_setup()
		ke.set_scanner_channel(chan+1)
		ke.set_integration_time(keithley_nplcs)
		ke.set_measurement ('R')
		R = ke.measure()
		ke.set_measurement ('V')
		
		qc.v[board] = 12
		
		q = q_setup()
		q.v[:] = 0
		
		device_type = q.chain[0]['device_type']
		
		
		if device_type == 'Q8iv':
			v_targs = [0.0, 0.5, 1.0, 3.0, 6.0, 9.0, 11.5]
			i_targs = [0.0, 0.5, 1.0, 3.0, 6.0, 9.0, 11.5]
		elif device_type == 'Q8b':
			v_targs = [0.0, 0.5, 1.0, 5.0, 10.0, 19.5]
			i_targs = []
		
		
		results = []
		
		q.v[chan] = 0
		
		print ("\nStepping...")
		t_start = time.time()
		for param,targs in [('v',v_targs),('i',i_targs)]:
			for targ in targs:
				print (" {:} = {:}".format(param,targ))
			
				results.append({})
			
				if param == 'v':
					q.v[chan] = targ
				elif param == 'i':
					q.i[chan] = targ
				else:
					raise RuntimeError()
				
				time.sleep(0.05)
			
				results[-1].update({'param':param})
				results[-1].update({'targ':targ})
				
				us,Is,vs = [],[],[]
				for _ in range(n_measurements):
					try:
						vs.append(ke.measure())
						us.append(q.v[chan])
						Is.append(q.i[chan])
					except:
						pass
				
				results[-1].update({'vmean':mean(vs)})
				results[-1].update({'vstd':std_dev(vs)})
				results[-1].update({'umean':mean(us)})
				results[-1].update({'ustd':std_dev(us)})
				results[-1].update({'imean':mean(Is)})
				results[-1].update({'istd':std_dev(Is)})
		
		
		q.i[chan] = 0.0
		q.v[chan] = 0.0
		
		# Write to file
		filename = make_filename("Precision")
		
		print ("\nSaving to file '{:}'".format(filename))
		f = open(os.path.expanduser(filename), "w+")
		
		
		f.write("{:},{:}\n".format(
									"Time",
									str(datetime.datetime.now())))
		f.write("{:},{:}\n".format(
									"Device ID",
									q.device_id))
		f.write("{:},{:}\n".format(
									"Device channel",
									chan))
		f.write("{:},{:}\n".format(
									"Keithley number of integrated powerline cycles",
									keithley_nplcs))
		f.write("{:},{:}\n".format(
									"Load resistance (Ohm)",
									R))	
									
									
		f.write("{:},{:},{:},{:},{:},{:},{:},{:}\n".format(
									"Parameter",
									"Target value",
									"Keithley voltage mean (V)",
									"Keithley voltage std dev (V)",
									"Onboard voltage mean(V)",
									"Onboard voltage std dev (V)",
									"Onboard current mean (mA)",
									"Onboard current std dev(mA)"))
		
		for j in range(len(results)):
			f.write("{:},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f}\n".format(
									results[j]['param'],
									results[j]['targ'],
									results[j]['vmean'],
									results[j]['vstd'],
									results[j]['umean'],
									results[j]['ustd'],
									results[j]['imean'],
									results[j]['istd']))
		
		f.close()
		
	if measurement == "longtermv" or measurement == "longtermi":
		from numpy import std as std_dev
		from numpy import mean as mean
		
		# Parameters
		
		board = 0
		chan = 0
		n_measurements = 100
		keithley_nplcs = 20.0
		v_targ = 6.0
		i_targ = 12.0
		t_period = 30.0
		
		# Script
		
		qc = qc_setup()
		qc.v[:] = 0
		
		ke = ke_setup()
		ke.set_scanner_channel(chan+1)
		ke.set_integration_time(keithley_nplcs)
		ke.set_measurement ('R')
		R = ke.measure()
		ke.set_measurement ('V')
		
		qc.v[board] = 12
		
		q = q_setup()
		q.v[:] = 0
		
		
		results = []
		
		
		print ("\nCollecting... started at {:}".format(str(datetime.datetime.now())))
		
		
		# Write to file
		if measurement == "longtermv":
			measurement_type = "Voltage"
		elif measurement == "longtermi":
			measurement_type = "Current"
			
		filename = make_filename("Longterm " + measurement_type)
		
		f = open(os.path.expanduser(filename), "w+")
		
		f.write("{:},{:}\n".format(
									"Start time",
									str(datetime.datetime.now())))
		f.write("{:},{:}\n".format(
									"Device ID",
									q.device_id))
		f.write("{:},{:}\n".format(
									"Device channel",
									chan))
		f.write("{:},{:}\n".format(
									"Keithley number of integrated powerline cycles",
									keithley_nplcs))
		f.write("{:},{:}\n".format(
									"Load resistance (Ohm)",
									R))	
		if measurement == "longtermv":
			f.write("{:},{:}\n".format(
									"Set voltage (V)",
									v_targ))	
			q.v[chan] = v_targ
		elif measurement == "longtermi":
			f.write("{:},{:}\n".format(
									"Set current (mA)",
									i_targ))
			q.i[chan] = i_targ	
									
									
		f.write("{:},{:},{:},{:},{:},{:},{:}\n".format(
									"Elapsed time (s)",
									"Keithley voltage mean (V)",
									"Keithley voltage std dev (V)",
									"Onboard voltage mean(V)",
									"Onboard voltage std dev (V)",
									"Onboard current mean (mA)",
									"Onboard current std dev(mA)"))
		f.flush()
		f.close()
		
		
		t_start = time.time()
		t_last = time.time()
		
		while True:
			
			
			while (time.time() - t_last) < t_period:
				try:
					time.sleep(0.1)
				except:
					q.v[:] = 0
					quit()
			
			t_last = time.time()
			print ("Measuring {:} repeats at {:}".format(n_measurements, str(datetime.datetime.now())))
			
			us,Is,vs = [],[],[]
			for _ in range(n_measurements):
				try:
					ke.measure()
					us.append(q.v[chan])
					ke.measure()
					Is.append(q.i[chan])
					vs.append(ke.measure())
				except:
					pass
			
			print (" writing to file")
			f = open(os.path.expanduser(filename), "a+")
			f.write("{:},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f}\n".format(
									time.time() - t_start,
									mean(vs),
									std_dev(vs),
									mean(us),
									std_dev(us),
									mean(Is),
									std_dev(Is)
									))
			f.flush()
			f.close()
		
		
		q.i[chan] = 0.0
		q.v[chan] = 0.0
	
	if measurement == "crosstalk":
		from numpy import std as std_dev
		from numpy import mean as mean
		
		# Parameters
		
		board = 0
		chan = 0
		other_chans = [1,2,3,4,5,6,7]
		keithley_nplcs = 10.0
		v_tests = [0.0, 0.5, 6.0, 11.5]
		n_vo_steps = 13
		vo_max = 12
		vo_min = 0
		
		# Script
		
		qc = qc_setup()
		qc.v[:] = 0
		qc.v[board] = 12
		
		q = q_setup()
		q.v[:] = 0
		
		qc.v[board] = 0 # disconnect
		
		ke = ke_setup()
		ke.set_integration_time(keithley_nplcs)
		
		
		
		
		# Write to file
		filename = make_filename("Crosstalk")
		print ("\nSaving to file '{:}'".format(filename))
		f = open(os.path.expanduser(filename), "w+")
		
		f.write("{:},{:}\n".format(
									"Time",
									str(datetime.datetime.now())))
		f.write("{:},{:}\n".format(
									"Device ID",
									q.device_id))
		f.write("{:},{:}\n".format(
									"Device channel",
									chan))
		f.write("{:},{:}\n".format(
									"Keithley number of integrated powerline cycles",
									keithley_nplcs))
		f.write("{:}".format(
									"Load resistances (Ohm)"))	
		ke.set_measurement ('R')
		for ch in range(8):
			ke.set_scanner_channel(ch+1)
			f.write(",{:}".format(ke.measure()))
		ke.set_measurement ('V')	
		ke.set_scanner_channel(chan+1)
		f.write("\n")
		
		qc.v[board] = 12 # reconnect
									
		f.write("{:},{:},{:},{:},{:},{:}\n".format(
									"Target channel nominal voltage (V)",
									"Target channel Keithley measured voltage (V)",
									"Target channel onboard measured voltage (V)",
									"Target channel onboard measured current (mA)",
									"Other channel number",
									"Other channel voltage (V)"))
		
		
		
		
		print ("\nStepping...")
		t_start = time.time()
		
		
		for vnom in v_tests:
			
			print ("Testing at Vnom = {:}".format(vnom))
			q.v[chan] = vnom
			
			for chano in other_chans:
				
				for n in range(n_vo_steps):
					vo = 1.*n*(vo_max-vo_min)/(n_vo_steps-1)
					q.v[chano] = vo
					time.sleep(0.05)
					
					ke.measure()
					ke.measure()
					v = ke.measure()
					u = q.v[chan]
					i = q.i[chan]
					
					f.write("{:.6f},{:.6f},{:.6f},{:.6f},{:},{:.6f}\n".format(
									vnom,
									v,
									u,
									i,
									chano,
									vo))
		
		
		q.i[:] = 0.0
		time.sleep(0.05)
		q.v[:] = 0.0
		
		f.close()
		
	if measurement == "scaletest":
		
		print ("\nPlug multimeter into corresponding pins on output connector, switch to front panel measurement.")
		
		# Parameters
		
		board = 0
		chan = 10
		
		v_tests = [6.0,12.0,20.0]
		vfull = 20.0
		
		

		# Script
		
		qc = qc_setup()
		qc.v[:] = 0
		time.sleep(2.0)
		qc.v[board] = 12
		
		q = q_setup(n_chs = 16, catch_errs = True)
		
		q.binary_mode = True
		
		q.v[:] = 0
		
		ke = ke_setup()
		
		
		print ("\nComparing voltages (ASCII)...")
		
		for vnom in v_tests:
			
			q.v[chan] = vnom
			time.sleep(0.05)
			v = ke.measure()
			
			print ("  {:6.3f} V ?= {:6.3f} V --> {:}".format(vnom, v, passfail(abs(v-vnom)/vnom<0.1) ) )
		
		
		
		
		# Binary commands
		
		def get_val(i):
			return bytearray([int(i/256),int(i)-int(i/256)*256])
		
		def get_v_cmd(v,vfull):
			i = int(0xFFFF*float(v/vfull))
			c = bytearray(b'\x81\x00\x00\x00')
			c.append(chan)
			c.extend(get_val(i))
			return c
		
		
		print ("\nComparing voltages (binary)...")
		
		for vnom in v_tests:
			q.serial_port.write(get_v_cmd(vnom,vfull))
			q.serial_port.flush()
			time.sleep(0.05)
			v = ke.measure()
			
			print ("  {:6.3f} V ?= {:6.3f} V --> {:}".format(vnom, v, passfail(abs(v-vnom)/vnom<0.1) ) )
		
		
		time.sleep(0.05)
		
		print("\nDone!\n")
		q.v[:] = 0.0
		qc.v[15] = 12
	
	if measurement == "icaltest":
		
		
		vmax = 20.0
		vstep = 0.5
		dwell_time = 0.1
		imax_frac_ifull = 0.5 # What fraction of ifull should imax be set to
		board = 0
		
		qc = qc_setup()
		qc.v[:] = 0
		
		ke = ke_setup()
		
		# Measure test resistances
		ke.set_integration_time(100.0)
		ke.set_measurement ('R')
		resistances = []
		
		sys.stdout.write("\r  Measuring test resistances:\n")
		
		for ch in range(8):
			ke.set_scanner_channel(ch+1)
			resistances.append(ke.measure())
			sys.stdout.write("    Channel {:}:  {:.3f} Ohm\n".format(ch, resistances[-1]))
		
		ke.set_integration_time(1.0)
		ke.set_measurement ('V')
		
		
		# Connect, setup driver
		qc.v[board] = 12
		
		q = q_setup()
# 		q.issue_command ('adct', None, '=', 3, n_lines_requested=1)
# 		q.issue_command ('adcn', None, '=', 16, n_lines_requested=1)
		
		# Set max current to fraction of ifull value
		if imax_frac_ifull > 1 or imax_frac_ifull < 0:
			raise RuntimeError
		ifull = float((q.ifull)[:-3])
		q.imax[:] = imax_frac_ifull*ifull
		
		# Copy stdout to log file
		sys.stdout = Tee('Temporary programming log.txt', 'w')
		
		calibrate_input_current_with_R(
				q, 
				ke, 
				resistances,
				n_chs = 8,
				vstep = vstep, 
				vmax = vmax, 
				ignore_limits = True,
				dwell_t = dwell_time,
				filename = 'test')
	
	if measurement == "calibratev":
		qc = qc_setup()
		qc.v[0] = 12
		qc.v[15] = 0
		q = q_setup(n_chs = 8, catch_errs = True)
		calibrate_voltage(q, timeout = 4.000)
	
	if measurement == "ioverinvestigate":
		ke = ke_setup()
		qc = qc_setup()
		qc.v[0] = 12
		qc.v[15] = 0
		q = q_setup(n_chs = 8, catch_errs = True)
		R_mean = 1.000
		S_mean_range = (0.9/R_mean, 1.1/R_mean)
		
		for i in range(5):
			print ("Step {:} of {:}".format(i,5))
			get_XY(
				q, 
				ke,
				xmin = 0.0,
				xmax = 12.0, 
				xstep = 0.05,
				set_variable = 'v',
				measured_variables = ['i','v','u'],
				measured_slope_tolerance_ranges = [S_mean_range,(0.9,1.1),(0.9,1.1)],
				filename = 'Temporary sweep V',
				plot = True,
				plot_deviation = True,
				rawi = True,
				rawv = True,
				random_order = False,
				dwell_t = 0.04,
				channels = [2,3],
				scan_keithley = True)
	
		


# def cli_batch_programme_and_log():
	# Things to get:
	#  Customer name
	#  Customer institute
	#  Customer invoice number
	#  Number of slots to program
	#  Module type to be programmed
	#  Hardware version of module to be programmed

	# Things to return:
	



# if __name__=="__main__":

# customer_name      = "Qontroller"
# customer_institute = "Qontrol"
# customer_quote_num = "Qontrol_test"

customer_name      = "Ali Elshaari"
customer_institute = "KTH"
customer_quote_num = "IQ278b"

note = "{name} {inst} {quot}".format(name=customer_name, inst=customer_institute, quot=customer_quote_num)


# test_cable (
# 	cable_type = "CAB12", 
# 	note = note,
# 	filename_prefix = '~/Qontrol Dropbox/Engineering/Outgoing devices/', 
# 	control_serial_port_glob = '/dev/tty.usbserial-FT2MCM5U')

batch_programme_and_log(
		slots_to_program = range(2),
		device_type = 'q8iv',
		customer_or_quote = note,
		force_preserve_nvm = False,
		calibrate_v = True,
		calibrate_i = True,
		high_quality_iv = True,              # should be True
		name = True,                         # will be Q8iv/Q8b-**** if it is set to False
		force_names = False,                 # usually be False, but if needed, use it like: ['Q8iv-036E']
		hang = False,
		flash_it = True,
		psu_voltage = [15.0, 15.0, 5.0],     # [29.0, 15.0, 5.0] for Q8b-24V
		cut_power_at_stop = True,
		target_serial_port_glob = '/dev/tty.usbserial-FT2MCR7R',
		control_serial_port_glob = '/dev/tty.usbserial-FT2MCM5U')


# batch_check(
# 			slots_to_check = range(7),
# 			sweep_v = True,
# 			sweep_i = False,
# 			target_serial_port_glob = '/dev/tty.usbserial-FT2MCR7R',
# 			control_serial_port_glob = '/dev/tty.usbserial-FT2MCM5U')





# Test for channel three overcurrent error
# get_documentation_data('ioverinvestigate')

# get_documentation_data('transients')
# get_documentation_data('vpower')
# get_documentation_data('crosstalk')

# get_documentation_data('smallvsig')
# get_documentation_data('smallisig')

# get_documentation_data('speed')
# get_documentation_data('longtermv')

# get_documentation_data('precision')

# get_documentation_data('calibratev')


# get_documentation_data('icaltest')

# get_documentation_data('scaletest')
