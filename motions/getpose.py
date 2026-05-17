import streamlit as st
from dynamixel_sdk import *
import pandas as pd
import time
import os
import json
import math

# --- 다이나믹셀 주소 설정 (X-시리즈 기준) ---
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116     
ADDR_PRESENT_POSITION = 132  

DXL_IDS = list(range(1, 23)) # 1~22번 모터 

MOTOR_GROUPS = {
    "왼쪽 다리": [1, 2, 3, 4, 5, 11],
    "오른쪽 다리": [6, 7, 8, 9, 10, 13],
    "허리": [12],
    "왼쪽 팔": [14, 15, 16, 17],
    "오른쪽 팔": [18, 19, 20, 21],
    "목": [22]
}

BASE_POSTURE = {
    1: 2088, 2: 1847, 3: 1467, 4: 1900, 5: 2066, 6: 2052, 7: 2276, 8: 2484, 9: 1797, 10: 2030,
    11: 2047, 12: 2005, 13: 2060, 14: 1931, 15: 2050, 16: 2016, 17: 2022, 18: 2139, 19: 2056, 20: 2089,
    21: 2058, 22: 1857
}

SYM_MAP = {
    1: 6, 2: 7, 3: 8, 4: 9, 5: 10, 11: 13,
    6: 1, 7: 2, 8: 3, 9: 4, 10: 5, 13: 11,
    14: 18, 15: 19, 16: 20, 17: 21,
    18: 14, 19: 15, 20: 16, 21: 17
}

st.set_page_config(page_title="G.O.A.T Motion Studio", layout="wide")
st.title("민규의 모션 추출기 (Pro Sequencer)")

@st.cache_resource
def init_dxl():
    ph = PortHandler('/dev/ttyUSB0')
    pkh = PacketHandler(2.0)
    if ph.openPort() and ph.setBaudRate(2000000):
        ph.clearPort()
        return ph, pkh
    return None, None

ph, pkh = init_dxl()
if ph is None:
    st.error("U2D2 연결 실패! 포트 권한을 확인하세요.")
    st.stop()

def load_poses_from_csv():
    if os.path.exists("goat_poses.csv") and os.path.getsize("goat_poses.csv") > 0:
        try:
            return pd.read_csv("goat_poses.csv").to_dict('records')
        except pd.errors.EmptyDataError:
            return []
    return []

def load_sequences_from_json():
    if os.path.exists("goat_sequences.json") and os.path.getsize("goat_sequences.json") > 0:
        with open("goat_sequences.json", "r") as f:
            return json.load(f)
    return {}

def move_motor(dxl_id):
    if st.session_state.torque_status[dxl_id] == "🟢 ON":
        target_pos = st.session_state[f"slider_{dxl_id}"]
        pkh.write4ByteTxRx(ph, dxl_id, ADDR_GOAL_POSITION, target_pos)

def apply_symmetry(src_id):
    if src_id not in SYM_MAP: return
    tgt_id = SYM_MAP[src_id]
    src_current = st.session_state[f"slider_{src_id}"]
    src_base = BASE_POSTURE[src_id]
    delta = src_current - src_base
    tgt_base = BASE_POSTURE[tgt_id]
    tgt_new = max(0, min(4095, tgt_base - delta))
    st.session_state[f"slider_{tgt_id}"] = tgt_new
    if st.session_state.torque_status[tgt_id] == "🔴 OFF":
        pkh.write1ByteTxRx(ph, tgt_id, ADDR_TORQUE_ENABLE, 1)
        st.session_state.torque_status[tgt_id] = "🟢 ON"
    pkh.write4ByteTxRx(ph, tgt_id, ADDR_GOAL_POSITION, tgt_new)

# 🔥 [200Hz + 가속도 0 댐핑]
def move_sequence_smoothly(sequence_names, durations_list, types_list, hz=200):
    if not sequence_names: return None
    dt = 1.0 / hz
    
    waypoints = []
    q_start = {}
    for dxl_id in DXL_IDS:
        pos, res, err = pkh.read4ByteTxRx(ph, dxl_id, ADDR_PRESENT_POSITION)
        q_start[dxl_id] = pos if res == COMM_SUCCESS else st.session_state[f"slider_{dxl_id}"]
    waypoints.append(q_start) 
    
    for name in sequence_names:
        pose_data = next((item for item in st.session_state.saved_poses if item["Name"] == name), None)
        if not pose_data: continue # 만약 포즈가 삭제됐다면 무시
        q_target = {}
        for dxl_id in DXL_IDS:
            val = pose_data.get(f"ID_{dxl_id}")
            q_target[dxl_id] = int(val) if pd.notna(val) else waypoints[-1][dxl_id]
        waypoints.append(q_target)
        
    N = len(waypoints) - 1 

    velocities = []
    accelerations = []
    for i in range(len(waypoints)):
        v_dict, a_dict = {}, {}
        for dxl_id in DXL_IDS:
            if i == 0 or i == len(waypoints) - 1 or types_list[i-1] == "Stop":
                v_dict[dxl_id] = 0.0 
                a_dict[dxl_id] = 0.0
            else:
                T_prev = float(durations_list[i-1])
                T_next = float(durations_list[i])
                v_dict[dxl_id] = (waypoints[i+1][dxl_id] - waypoints[i-1][dxl_id]) / (T_prev + T_next)
                a_dict[dxl_id] = 0.0 
        velocities.append(v_dict)
        accelerations.append(a_dict)

    groupSyncWrite = GroupSyncWrite(ph, pkh, ADDR_GOAL_POSITION, 4)

    # 각 구간마다 5차 다항식 보간으로 부드러운 궤적 생성
    # 포즈 123과 포즈 456를 잇는 구간에서는 포즈 123의 끝과 포즈 456의 시작이 자연스럽게 연결되도록, 포즈 123의 마지막 속도를 포즈 456의 첫 속도와 같게 설정해서 보간합니다.
    # 속도의 기준은 중앙차분법으로 계산하되, 첫 구간과 마지막 구간은 정지 상태로 시작/끝나도록 속도를 0으로 설정합니다.
    # 실제 실행 시 포즈 123456은 포즈 123 -[부드러운 연결]-> 포즈 456 형태로 재생되며, 포즈 123의 마지막과 포즈 456의 처음이 자연스럽게 이어지면서도, 각 포즈에서 설정한 시간 동안 부드러운 궤적이 만들어집니다.
    # stop을 설정한 구간은 완전히 정지 상태로 궤적이 만들어지도록, 해당 구간의 끝 속도를 0으로 강제 설정합니다.

    for i in range(N):
        q0, qf = waypoints[i], waypoints[i+1]
        v0, vf = velocities[i], velocities[i+1]
        a0, af = accelerations[i], accelerations[i+1]
        T = float(durations_list[i])
        steps_per_pose = max(1, int(T * hz))
        
        c0, c1, c2, c3, c4, c5 = {}, {}, {}, {}, {}, {}
        for dxl_id in DXL_IDS:
            dq = qf[dxl_id] - q0[dxl_id]
            _v0, _vf = v0[dxl_id], vf[dxl_id]
            _a0, _af = a0[dxl_id], af[dxl_id]
            c0[dxl_id] = q0[dxl_id]
            c1[dxl_id] = _v0
            c2[dxl_id] = _a0 / 2.0 
            c3[dxl_id] = (10*dq - (6*_v0 + 4*_vf)*T - (1.5*_a0 - 0.5*_af)*(T**2)) / (T**3)
            c4[dxl_id] = (-15*dq + (8*_v0 + 7*_vf)*T + (1.5*_a0 - _af)*(T**2)) / (T**4)
            c5[dxl_id] = (6*dq - 3*(_v0 + _vf)*T - (0.5*_a0 - 0.5*_af)*(T**2)) / (T**5)

        for step in range(1, steps_per_pose + 1):
            loop_start = time.time()
            t = step * dt
            for dxl_id in DXL_IDS:
                current_q = c0[dxl_id] + c1[dxl_id]*t + c2[dxl_id]*(t**2) + c3[dxl_id]*(t**3) + c4[dxl_id]*(t**4) + c5[dxl_id]*(t**5)
                val = int(current_q)
                param = [DXL_LOBYTE(DXL_LOWORD(val)), DXL_HIBYTE(DXL_LOWORD(val)),
                         DXL_LOBYTE(DXL_HIWORD(val)), DXL_HIBYTE(DXL_HIWORD(val))]
                groupSyncWrite.addParam(dxl_id, param)
                
            groupSyncWrite.txPacket()
            groupSyncWrite.clearParam()
            elapsed = time.time() - loop_start
            time.sleep(max(0.0, dt - elapsed))
            
    return next((item for item in st.session_state.saved_poses if item["Name"] == sequence_names[-1]), None)

def export_to_csv(m_name, sequence_names, durations_list, types_list, hz=200):
    if not sequence_names: return ""
    dt = 1.0 / hz
    
    waypoints = []
    first_pose = next((item for item in st.session_state.saved_poses if item["Name"] == sequence_names[0]), None)
    if not first_pose: return ""
    q_start = {dxl_id: int(first_pose.get(f"ID_{dxl_id}", 2048)) for dxl_id in DXL_IDS}
    waypoints.append(q_start)
    
    for name in sequence_names:
        pose_data = next((item for item in st.session_state.saved_poses if item["Name"] == name), None)
        q_target = {dxl_id: int(pose_data.get(f"ID_{dxl_id}", waypoints[-1][dxl_id])) for dxl_id in DXL_IDS}
        waypoints.append(q_target)
        
    N = len(waypoints) - 1 

    velocities, accelerations = [], []
    for i in range(len(waypoints)):
        v_dict, a_dict = {}, {}
        for dxl_id in DXL_IDS:
            if i == 0 or i == len(waypoints) - 1 or types_list[i-1] == "Stop":
                v_dict[dxl_id], a_dict[dxl_id] = 0.0, 0.0
            else:
                T_prev = float(durations_list[i-1])
                T_next = float(durations_list[i])
                v_dict[dxl_id] = (waypoints[i+1][dxl_id] - waypoints[i-1][dxl_id]) / (T_prev + T_next)
                a_dict[dxl_id] = 0.0
        velocities.append(v_dict)
        accelerations.append(a_dict)

    csv_data = []
    for i in range(N):
        q0, qf = waypoints[i], waypoints[i+1]
        v0, vf = velocities[i], velocities[i+1]
        a0, af = accelerations[i], accelerations[i+1]
        T = float(durations_list[i])
        steps_per_pose = max(1, int(T * hz))
        
        c0, c1, c2, c3, c4, c5 = {}, {}, {}, {}, {}, {}
        for dxl_id in DXL_IDS:
            dq = qf[dxl_id] - q0[dxl_id]
            _v0, _vf = v0[dxl_id], vf[dxl_id]
            _a0, _af = a0[dxl_id], af[dxl_id]
            c0[dxl_id] = q0[dxl_id]
            c1[dxl_id] = _v0
            c2[dxl_id] = _a0 / 2.0 
            c3[dxl_id] = (10*dq - (6*_v0 + 4*_vf)*T - (1.5*_a0 - 0.5*_af)*(T**2)) / (T**3)
            c4[dxl_id] = (-15*dq + (8*_v0 + 7*_vf)*T + (1.5*_a0 - _af)*(T**2)) / (T**4)
            c5[dxl_id] = (6*dq - 3*(_v0 + _vf)*T - (0.5*_a0 - 0.5*_af)*(T**2)) / (T**5)

        for step in range(1, steps_per_pose + 1):
            t = step * dt
            row = []
            for dxl_id in DXL_IDS:
                current_q = c0[dxl_id] + c1[dxl_id]*t + c2[dxl_id]*(t**2) + c3[dxl_id]*(t**3) + c4[dxl_id]*(t**4) + c5[dxl_id]*(t**5)
                row.append(round((current_q - 2048.0) * (math.pi / 2048.0), 6))
            csv_data.append(row)
            
    file_name = f"motion_{m_name}.csv"
    pd.DataFrame(csv_data).to_csv(file_name, index=False, header=False)
    return file_name

# ==========================================
# 0. 초기화
# ==========================================
if 'initialized' not in st.session_state:
    st.session_state.torque_status = {idx: "🔴 OFF" for idx in DXL_IDS}
    st.session_state.saved_poses = load_poses_from_csv()
    st.session_state.saved_sequences = load_sequences_from_json() 
    
    for dxl_id in DXL_IDS:
        pos, res, err = pkh.read4ByteTxRx(ph, dxl_id, ADDR_PRESENT_POSITION)
        st.session_state[f"slider_{dxl_id}"] = pos if res == COMM_SUCCESS else 2048
        
    st.session_state.timeline = [] # 타임라인 편집기 초기화
    st.session_state.initialized = True

if 'pose_to_apply' in st.session_state:
    for dxl_id in DXL_IDS:
        val = st.session_state.pose_to_apply.get(f"ID_{dxl_id}")
        if pd.notna(val): st.session_state[f"slider_{dxl_id}"] = int(val)
    del st.session_state.pose_to_apply

# ==========================================
# 1. 상단 마스터 컨트롤 & 저장 & 대칭
# ==========================================
st.subheader("마스터 제어 및 포즈 캡처")
col_m1, col_m2, col_m3, col_m4 = st.columns([1, 1, 1.5, 1])
with col_m1:
    if st.button("🟢 ALL TORQUE ON", width='stretch', type="primary"):
        for dxl_id in DXL_IDS:
            pos, res, err = pkh.read4ByteTxRx(ph, dxl_id, ADDR_PRESENT_POSITION)
            if res == COMM_SUCCESS: st.session_state[f"slider_{dxl_id}"] = pos
            pkh.write1ByteTxRx(ph, dxl_id, ADDR_TORQUE_ENABLE, 1)
            st.session_state.torque_status[dxl_id] = "🟢 ON"
        st.rerun()
with col_m2:
    if st.button("🔴 ALL TORQUE OFF", width='stretch'):
        for dxl_id in DXL_IDS:
            pkh.write1ByteTxRx(ph, dxl_id, ADDR_TORQUE_ENABLE, 0)
            st.session_state.torque_status[dxl_id] = "🔴 OFF"
        st.rerun()
with col_m3:
    pose_name = st.text_input("포즈 이름", value=f"포즈_{len(st.session_state.saved_poses)+1}", label_visibility="collapsed")
with col_m4:
    if st.button("이름 붙여서 캡처!", width='stretch', type="secondary"):
        current_pose = {"Name": pose_name}
        for dxl_id in DXL_IDS:
            pos, res, err = pkh.read4ByteTxRx(ph, dxl_id, ADDR_PRESENT_POSITION)
            if res == COMM_SUCCESS:
                current_pose[f"ID_{dxl_id}"] = pos
                st.session_state[f"slider_{dxl_id}"] = pos 
            else:
                current_pose[f"ID_{dxl_id}"] = st.session_state[f"slider_{dxl_id}"]
        st.session_state.saved_poses.append(current_pose)
        pd.DataFrame(st.session_state.saved_poses).to_csv("goat_poses.csv", index=False)
        st.success(f"[{pose_name}] 캡처 완료!")
        time.sleep(0.5); st.rerun()

if st.session_state.saved_poses:
    pose_names = [p["Name"] for p in st.session_state.saved_poses]
    sym_col1, sym_col2, sym_col3 = st.columns([2, 2, 1])
    with sym_col1:
        target_pose_name = st.selectbox("🪞 원본 포즈 선택", options=pose_names, key="sym_target")
    with sym_col2:
        new_sym_name = st.text_input("대칭 포즈 이름", value=f"{target_pose_name}_대칭", key="sym_new_name")
    with sym_col3:
        st.write("") 
        if st.button("대칭 포즈 생성!", width='stretch'):
            src_pose = next(p for p in st.session_state.saved_poses if p["Name"] == target_pose_name)
            new_pose = {"Name": new_sym_name}
            for dxl_id in DXL_IDS:
                if f"ID_{dxl_id}" not in src_pose: continue
                src_val = src_pose[f"ID_{dxl_id}"]
                if dxl_id in SYM_MAP:
                    tgt_id = SYM_MAP[dxl_id]
                    tgt_new = BASE_POSTURE[tgt_id] - (src_val - BASE_POSTURE[dxl_id])
                    new_pose[f"ID_{tgt_id}"] = max(0, min(4095, int(tgt_new)))
                else:
                    new_pose[f"ID_{dxl_id}"] = src_val 
            st.session_state.saved_poses.append(new_pose)
            pd.DataFrame(st.session_state.saved_poses).to_csv("goat_poses.csv", index=False)
            st.success(f"[{new_sym_name}] 생성 완료!")
            time.sleep(0.5); st.rerun()
st.divider()

# ==========================================
# 2. 부위별 모터 제어 (생략 없이 기존과 동일)
# ==========================================
st.subheader("부위별 모터 미세 조절")
for group_name, group_ids in MOTOR_GROUPS.items():
    with st.expander(f"{group_name} ({len(group_ids)}개 관절)", expanded=False):
        g_col1, g_col2, g_col3 = st.columns([1, 1, 4])
        if g_col1.button(f"🟢 {group_name} 전체 ON", key=f"g_on_{group_name}"):
            for dxl_id in group_ids:
                pos, res, err = pkh.read4ByteTxRx(ph, dxl_id, ADDR_PRESENT_POSITION)
                if res == COMM_SUCCESS: st.session_state[f"slider_{dxl_id}"] = pos
                pkh.write1ByteTxRx(ph, dxl_id, ADDR_TORQUE_ENABLE, 1)
                st.session_state.torque_status[dxl_id] = "🟢 ON"
            st.rerun()
        if g_col2.button(f"🔴 {group_name} 전체 OFF", key=f"g_off_{group_name}"):
            for dxl_id in group_ids:
                pkh.write1ByteTxRx(ph, dxl_id, ADDR_TORQUE_ENABLE, 0)
                st.session_state.torque_status[dxl_id] = "🔴 OFF"
            st.rerun()
        st.write("---")
        for dxl_id in group_ids:
            c1, c2, c3, c4 = st.columns([1.2, 1.2, 0.6, 4])
            c1.write(f"**ID {dxl_id}** : {st.session_state.torque_status[dxl_id]}")
            btn1, btn2 = c2.columns(2)
            if btn1.button("ON", key=f"on_{dxl_id}"):
                pos, res, err = pkh.read4ByteTxRx(ph, dxl_id, ADDR_PRESENT_POSITION)
                if res == COMM_SUCCESS: st.session_state[f"slider_{dxl_id}"] = pos
                pkh.write1ByteTxRx(ph, dxl_id, ADDR_TORQUE_ENABLE, 1)
                st.session_state.torque_status[dxl_id] = "🟢 ON"
                st.rerun()
            if btn2.button("OFF", key=f"off_{dxl_id}"):
                pkh.write1ByteTxRx(ph, dxl_id, ADDR_TORQUE_ENABLE, 0)
                st.session_state.torque_status[dxl_id] = "🔴 OFF"
                st.rerun()
            if dxl_id in SYM_MAP:
                c3.button("🔄", key=f"sym_{dxl_id}", on_click=apply_symmetry, args=(dxl_id,))
            is_disabled = (st.session_state.torque_status[dxl_id] == "🔴 OFF")
            c4.slider(f"Angle {dxl_id}", min_value=0, max_value=4095, key=f"slider_{dxl_id}", on_change=move_motor, args=(dxl_id,), label_visibility="collapsed", disabled=is_disabled)

# ==========================================
# 3. 🎬 타임라인 편집기 (포즈 & 매크로 혼합 조립)
# ==========================================
if st.session_state.saved_poses:
    st.divider()
    st.subheader("🎬 타임라인 편집기 (포즈 & 매크로 조립)")
    
    macro_names = list(st.session_state.saved_sequences.keys())
    
    # -----------------------------------
    # A. 타임라인에 아이템 끼워넣기 UI
    # -----------------------------------
    col_type, col_item, col_idx, col_add = st.columns([1.5, 3, 1, 1.5])
    
    with col_type:
        add_type = st.radio("추가할 종류", ["단일 포즈", "매크로 콤보"], horizontal=True, label_visibility="collapsed")
    with col_item:
        if add_type == "단일 포즈":
            selected_item = st.selectbox("항목 선택", options=pose_names, label_visibility="collapsed")
        else:
            if not macro_names:
                st.warning("저장된 매크로가 없습니다.")
                selected_item = None
            else:
                selected_item = st.selectbox("항목 선택", options=macro_names, label_visibility="collapsed")
    
    with col_idx:
        # 현재 리스트 길이 + 1 이 기본값 (맨 뒤에 추가)
        insert_idx = st.number_input("추가할 위치", min_value=1, max_value=len(st.session_state.timeline)+1, value=len(st.session_state.timeline)+1)
    
    with col_add:
        if st.button("➕ 타임라인에 추가", width='stretch', type="primary"):
            if selected_item:
                target_idx = insert_idx - 1 # 리스트 인덱스에 맞게 보정
                
                if add_type == "단일 포즈":
                    # 기본값 세팅해서 넣기
                    st.session_state.timeline.insert(target_idx, {"Step": "-", "Name": selected_item, "Duration": 0.1, "Type": "Smooth"})
                else:
                    # 매크로를 분해해서 통째로 끼워넣기!
                    m_data = st.session_state.saved_sequences[selected_item]
                    m_seq = m_data["seq"]
                    m_durs = m_data["durations"]
                    m_types = m_data.get("types", ["Smooth"] * len(m_seq))
                    
                    # 매크로 내의 루프 처리는 저장될 때 이미 풀려있음
                    for i in range(len(m_seq)):
                        st.session_state.timeline.insert(target_idx + i, {"Step": "-", "Name": m_seq[i], "Duration": m_durs[i], "Type": m_types[i]})
                        
                st.rerun()

    # -----------------------------------
    # B. 편집 가능한 타임라인 표 (Data Editor)
    # -----------------------------------
    st.write("---")
    if not st.session_state.timeline:
        st.info("타임라인이 비어있습니다. 위에서 포즈나 매크로를 추가해주세요.")
    else:
        # 넘버링 업데이트
        for idx, row in enumerate(st.session_state.timeline):
            row["Step"] = str(idx + 1)
            
        df_timeline = pd.DataFrame(st.session_state.timeline)
        
        # 편집기 (여기서 시간/타입 수정 가능, 왼쪽 체크박스로 삭제 가능)
        st.write("✏️ **시간(초)과 타입(Smooth/Stop)을 더블클릭해서 수정할 수 있습니다.** (왼쪽 체크박스 선택 후 Delete 키로 삭제 가능)")
        
        edited_df = st.data_editor(
            df_timeline,
            column_config={
                "Step": st.column_config.TextColumn("순서", disabled=True, width="small"),
                "Name": st.column_config.TextColumn("포즈 이름", disabled=True),
                "Duration": st.column_config.NumberColumn("시간 (초)", min_value=0.01, max_value=5.0, format="%.2f", step=0.01),
                "Type": st.column_config.SelectboxColumn("타입", options=["Smooth", "Stop"], required=True)
            },
            num_rows="dynamic", # 줄 삭제 가능
            hide_index=True,
            width='stretch'
        )
        
        # 편집된 데이터 세션에 즉시 반영
        st.session_state.timeline = edited_df.to_dict('records')
        
        st.write("---")
        
        # -----------------------------------
        # C. 재생 & 루프 & 매크로 저장
        # -----------------------------------
        c_play1, c_play2, c_save = st.columns([1.5, 1.5, 2])
        
        # 데이터 분리
        t_names = [row["Name"] for row in st.session_state.timeline]
        t_durs = [row["Duration"] for row in st.session_state.timeline]
        t_types = [row["Type"] for row in st.session_state.timeline]

        with c_play1:
            if st.button("▶️ 1회 재생 (테스트)", width='stretch'):
                status_text = st.empty()
                for dxl_id in DXL_IDS:
                    pkh.write1ByteTxRx(ph, dxl_id, ADDR_TORQUE_ENABLE, 1)
                    st.session_state.torque_status[dxl_id] = "🟢 ON"
                
                status_text.info(f"재생 중... (총 {len(t_names)}개 스텝)")
                final_pose = move_sequence_smoothly(t_names, t_durs, t_types, hz=200)
                if final_pose: st.session_state.pose_to_apply = final_pose
                status_text.success(f"완료! ({sum(t_durs):.1f}초)")
                time.sleep(1); st.rerun()

        with c_play2:
            with st.popover("🔁 루프 재생", width='stretch'):
                loop_cnt = st.number_input("현재 타임라인을 몇 번 반복할까요?", min_value=2, max_value=100, value=2)
                # 매크로에서 매크로로 넘어갈 때 이어주는 부분은 모두 Smooth로 처리하기 위해 타입 강제 조정할 수 있지만, 편집기에서 설정한 대로 둠.
                if st.button(f"루프 {loop_cnt}회 시작!", width='stretch', type="primary"):
                    loop_names = t_names * loop_cnt
                    loop_durs = t_durs * loop_cnt
                    loop_types = t_types * loop_cnt
                    
                    status_text = st.empty()
                    for dxl_id in DXL_IDS:
                        pkh.write1ByteTxRx(ph, dxl_id, ADDR_TORQUE_ENABLE, 1)
                    
                    status_text.info(f"{loop_cnt}회 루프 재생 중... (총 {len(loop_names)} 스텝)")
                    final_pose = move_sequence_smoothly(loop_names, loop_durs, loop_types, hz=200)
                    if final_pose: st.session_state.pose_to_apply = final_pose
                    status_text.success("루프 완료!")
                    time.sleep(1); st.rerun()

        with c_save:
            with st.popover("💾 현재 타임라인을 새 매크로로 저장", width='stretch'):
                new_m_name = st.text_input("새 매크로 이름")
                if st.button("매크로 저장", width='stretch'):
                    if new_m_name and t_names:
                        st.session_state.saved_sequences[new_m_name] = {
                            "seq": t_names,
                            "durations": t_durs,
                            "types": t_types
                        }
                        with open("goat_sequences.json", "w") as f:
                            json.dump(st.session_state.saved_sequences, f)
                        st.success(f"[{new_m_name}] 저장 완료! (이전 매크로 관리 탭에서 확인 가능)")
                        time.sleep(1); st.rerun()

# ==========================================
# 4. 저장된 매크로 관리 및 CSV 추출
# ==========================================
if st.session_state.get('saved_sequences'):
    st.divider()
    st.subheader("저장된 매크로 관리")
    
    macro_cols = st.columns(3)
    for i, (m_name, m_data) in enumerate(list(st.session_state.saved_sequences.items())):
        with macro_cols[i % 3]:
            with st.container(border=True):
                st.write(f"**{m_name}**")
                st.caption(f"총 {len(m_data['seq'])} 스텝 ({sum(m_data['durations']):.1f}초)")
                
                btn_c1, btn_c2, btn_c3 = st.columns([1, 1, 1])
                m_types = m_data.get("types", ["Smooth"] * len(m_data["seq"]))
                
                if btn_c1.button("▶️ 재생", key=f"play_m_{i}", width='stretch'):
                    for dxl_id in DXL_IDS:
                        pkh.write1ByteTxRx(ph, dxl_id, ADDR_TORQUE_ENABLE, 1)
                        st.session_state.torque_status[dxl_id] = "🟢 ON"
                    st.toast(f"[{m_name}] 매크로 재생 시작!")
                    final_pose = move_sequence_smoothly(m_data['seq'], m_data['durations'], m_types, hz=200)
                    if final_pose: st.session_state.pose_to_apply = final_pose
                    st.rerun()
                    
                if btn_c2.button("🗑️ 삭제", key=f"del_m_{i}", width='stretch'):
                    del st.session_state.saved_sequences[m_name]
                    with open("goat_sequences.json", "w") as f:
                        json.dump(st.session_state.saved_sequences, f)
                    st.rerun()
                    
                if btn_c3.button("📄 CSV", key=f"code_m_{i}", width='stretch', type="secondary"):
                    saved_file_name = export_to_csv(m_name, m_data['seq'], m_data['durations'], m_types, hz=200)
                    st.success(f"{saved_file_name} 저장 완료!")