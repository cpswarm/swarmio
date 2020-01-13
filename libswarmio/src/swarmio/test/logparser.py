#!/usr/bin/env python3
import os
import sys
import subprocess
import time

script_path = os.path.dirname(os.path.realpath(sys.argv[0]))

try:
    import junitparser
except ImportError:
    import subprocess
    print("junitparser module was not found, installing...")
    subprocess.call([os.path.join('.', script_path, 'test_installer.sh')])

from junitparser import TestCase, TestSuite, JUnitXml, Skipped, Element

class Result(Skipped):
    _tag = 'result'
    _inlinemsg = None
    _message = None
    _type = None

    def __init__(self, inlinemsg=None, message=None, type_=None):
        if inlinemsg is None:
            inlinemsg = self._inlinemsg
        if inlinemsg is not None:
            self._opentag = '<' + self._tag + '>'
            self._closetag = '</' + self._tag + '>'
            self._elem = Element.fromstring(self._opentag + inlinemsg + self._closetag)._elem
        else:
            if message is None:
                message = self._message
            if type_ is None:
                type_ = self._type
            super().__init__(message=message, type_=type_)
        self.message = message
        self.type = type_

class Failure(Result):
    _tag = 'failure'
    _message = 'error'
    _type = 'failed'

class Success(Result):
    _message = 'success'
    _type = 'ok'

BIN_NAME = os.path.join('.', script_path, 'swarmio-simulator')
PROCESS_START_DELAY_S = 1
PROCESSES_RUN_S = 1
PROCESS_TIMEOUT_S = 2

files = os.listdir('.')
proc1 = subprocess.Popen(BIN_NAME, stdin=subprocess.PIPE, stdout=subprocess.PIPE)
time.sleep(PROCESS_START_DELAY_S)

new_files = os.listdir('.')
proc1_log = [f for f in new_files if f not in files][0]

files = os.listdir('.')
proc2 = subprocess.Popen(BIN_NAME, stdin=subprocess.PIPE, stdout=subprocess.PIPE)

time.sleep(PROCESSES_RUN_S)

# Create cases
program_starting_case = TestCase('program_starting_case')
discovery_case = TestCase('discovery_case')

try:
    out, err = proc1.communicate(input=b'\n', timeout=PROCESS_TIMEOUT_S)
    proc1_out = out.decode('unicode_escape').split('\n')
    out, err = proc2.communicate(input=b'\n', timeout=PROCESS_TIMEOUT_S)
    proc2_out = out.decode('unicode_escape').split('\n')

    new_files = os.listdir('.')
    proc2_log = [f for f in new_files if f not in files][0]

    def parse_log(log_fname, other_node):
        found_other_node = False
        with open(log_fname) as fp:
            for i, line in enumerate(fp):
                if i > 2:
                    if '\t' in line:
                        spl = line.split('\t')
                        ltime = spl[0]
                        lmsg = spl[1]
                        if other_node in line:
                            found_other_node = True
                    else:
                        ltime = ''
                        lmsg = line
                elif 'log format' in line.lower():
                    s = line.find('[') + 1
                    e = line.find(']')
                    log_frmt = line[s:e]
        return found_other_node

    uuidpos = proc2_out[0].find('UUID: ') + len('UUID: ')
    proc1_found_proc2 = parse_log(proc1_log, proc2_out[0][uuidpos:])
    proc2_found_proc1 = parse_log(proc2_log, proc1_out[0][uuidpos:])

    # Set case results
    program_starting_case.result = Success()
    if proc1_found_proc2 and proc2_found_proc1:
        discovery_case.result = Success()
    else:
        discovery_case.result = Failure(message='discovery error')
except Exception:
    if proc1.poll() == None:
        proc1.kill()
        proc1.communicate()
    if proc2.poll() == None:
        proc2.kill()
        proc2.communicate()
    # Set case results
    program_starting_case.result = Failure(message='program crash error')
    discovery_case.result = Failure(message='discovery error') 

# Create suite and add cases
suite = TestSuite('testsuite')
#suite.add_property('build', '55')
suite.add_testcase(program_starting_case)
suite.add_testcase(discovery_case)

# Add suite to JunitXml
xml = JUnitXml()
xml.add_testsuite(suite)

test_reporst_dir = os.path.join('tests', 'test-reports')
if not os.path.exists(test_reporst_dir):
    os.makedirs(test_reporst_dir)

xml.write(os.path.join(test_reporst_dir, 'result.xml'), pretty=True)
