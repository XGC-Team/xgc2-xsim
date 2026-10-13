#!/usr/bin/env python3
"""Small real-GPU fixture. Run only in a private network-none GPU container."""
import argparse, json, os, signal, socket, subprocess, time, xmlrpc.client
from pathlib import Path
from native_http import NativeProvider, UnixHTTP, assert_retired_ros_surfaces_absent

def main():
    p=argparse.ArgumentParser();p.add_argument('--xsim',required=True);p.add_argument('--module');p.add_argument('--output',required=True);a=p.parse_args()
    if a.module:os.environ['XGC2_SENSING_GPU_LIBRARY']=str(Path(a.module).resolve())
    out=Path(a.output);out.mkdir(parents=True,exist_ok=True)
    out.chmod(0o700)
    for name in ("ros-home", "ros-log"):
        (out / name).mkdir(mode=0o700, exist_ok=True)
    with socket.socket() as s:s.bind(('127.0.0.1',0));port=s.getsockname()[1]
    os.environ.update(ROS_MASTER_URI=f'http://127.0.0.1:{port}',ROS_IP='127.0.0.1',ROS_HOSTNAME='127.0.0.1',ROS_HOME=str(out/'ros-home'),ROS_LOG_DIR=str(out/'ros-log'))
    children=[];logs=[]
    def spawn(cmd,log):
        f=open(out/log,'w');logs.append(f);proc=subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT,start_new_session=True);children.append(proc);return proc
    def wait(fn,label):
        end=time.monotonic()+10
        while time.monotonic()<end:
            try:
                if fn():return
            except OSError:pass
            time.sleep(.02)
        raise AssertionError(label)
    try:
        spawn(['roscore','-p',str(port)],'master.log')
        master=xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI'])
        wait(lambda:master.getPid('/gpu_fixture')[0]==1,'master')
        import rospy
        from sensor_msgs.msg import PointCloud2
        rospy.set_param('/use_sim_time',True);rospy.init_node('gpu_fixture',disable_signals=True)
        entities=[]
        for i,(h,v) in enumerate(((120,40),(240,80))):
            entities.append({'name':'gpu_'+str(i),'kind':'mecanum','position':[0,i,0],
                'sensor':{'backend':'gpu','mode':'raycast','h_res':h,'v_res':v,'h_fov_deg':90,'v_fov_deg':30,'min_range':.2,'range':20,'rate_hz':10,'point_cover_spacing':.1}})
        cfg={'instance_id':'gpu-fixture','epoch_ns':1700000000000000000,'publish_clock':True,
             'scene':{'surface_spacing':.1,'obstacles':[{'type':'box','position':[5,0,0],'size':[1,6,4]}]},'entities':[]}
        (out/'world.json').write_text(json.dumps(cfg))
        server=spawn([a.xsim,'--config',str(out/'world.json'),'--socket',str(out/'world.sock'),'--target-id','gpu:world'],'xsim.log')
        received=[[],[]]
        subs=[rospy.Subscriber('/gpu_'+str(i)+'/cloud',PointCloud2,lambda m,i=i:received[i].append((m.header.stamp.to_nsec(),m.width,m.point_step,m.header.frame_id)),queue_size=10) for i in range(2)]
        wait(lambda:(out/'world.sock').exists(),'socket')
        def http(method,path,body=None):
            c=UnixHTTP(str(out/'world.sock'));c.request(method,path,json.dumps(body) if body else None,{'Content-Type':'application/json'});r=c.getresponse();code=r.status;j=json.loads(r.read());c.close();return code,j
        before=http('GET','/status')[1]['steps']
        preparation_steps=[]
        for i,spec in enumerate(entities):
            code,receipt=http('POST','/entities',{'instance_id':'gpu-fixture','request_id':'add-'+str(i),'timeout_ms':5000,'entity':spec});assert code==202
            def applied():
                receipt=http('GET','/requests/add-'+str(i))[1]
                assert receipt['phase'] not in ('cancelled','failed'),receipt
                return receipt['phase']=='applied' and receipt['result']['success']
            wait(applied,'GPU prepared before world add')
            preparation_steps.append(http('GET','/status')[1]['steps']-before)
        assert preparation_steps[0]>0
        for i in range(2):
            r=NativeProvider(str(out/'world.sock'),'gpu_'+str(i)).start(0);assert r['success']
        wait(lambda:all(len(x)>=3 and x[-1][1]>0 for x in received),'real GPU points from both resolutions')
        code,description,graph=master.getSystemState('/gpu_fixture');assert code==1,description
        graph_report=assert_retired_ros_surfaces_absent(graph,('gpu_0','gpu_1'),('gpu_0','gpu_1'))
        assert all(x[-1][2]==16 for x in received),received
        assert all(len({sample[1] for sample in sensor})==1 for sensor in received),received
        conn=UnixHTTP(str(out/'world.sock'));conn.request('GET','/status');status=json.loads(conn.getresponse().read());conn.close()
        assert len(status['sensors'])==2 and all(s['backend']=='gpu' and not s['errors'] and s['scans']>=3 for s in status['sensors']),status
        comm=[x.read_text().strip() for x in Path('/proc/'+str(server.pid)+'/task').glob('*/comm')]
        assert comm.count('xsim-gpu')==1 and 'xsim-sensor' not in comm,comm
        os.killpg(server.pid,signal.SIGTERM);assert server.wait(timeout=10)==0
        text=(out/'xsim.log').read_text();assert text.count('GL_VENDOR=')==1,text
        gl=[line for line in text.splitlines() if line.startswith(('GL_VENDOR=','GL_RENDERER=','GL_VERSION='))]
        assert any('NVIDIA' in line or 'AMD' in line or 'Intel' in line for line in gl),gl
        result={'result':'PASS','gl':gl,'map_upload_context_count':1,'sensor_count':2,'owned_gpu_threads':1,'cpu_sensor_threads':0,
                'observations':received,'world_steps_during_preparation':preparation_steps,'status':status,'ros_graph':graph_report,'shutdown_exit':0,'scope':'Original spherical-nearest GPU, two resolutions sharing one context/map; small static scene only.'}
        (out/'result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
    finally:
        for proc in reversed(children):
            if proc.poll() is None:
                os.killpg(proc.pid,signal.SIGTERM)
                try:proc.wait(timeout=8)
                except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
        for f in logs:f.close()
if __name__=='__main__':main()
