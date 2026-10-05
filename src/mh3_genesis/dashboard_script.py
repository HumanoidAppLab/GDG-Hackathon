"""Provide the documented browser controller bundled with the dashboard."""

SCRIPT = r"""
const controls = new Map();
const cameraStatus = document.querySelector('#camera-status');
let built = false;
let drive = 'stop';
let cameraSources = [];
let feedsBusy = false;
let ikBuilt = false;
let ikSolving = false;
let currentQuat = null;
let currentEuler = null;
let currentPos = null;
let currentPosClean = true;
let currentOrientClean = true;

/**
 * Send a dashboard command to the local simulation server.
 * @param {string} path - Command endpoint.
 * @param {object} body - JSON command payload.
 * @returns {Promise<Response>} The HTTP response promise.
 */
function post(path, body)
{
    return fetch(path, {
        method: 'POST',
        headers: {'Content-Type': 'application/json', 'X-Control-Token': TOKEN},
        body: JSON.stringify(body),
    });
}

/**
 * Choose the dashboard panel for a public joint name.
 * @param {string} name - Public joint name.
 * @returns {string} The panel title.
 */
function group(name)
{
    if (/^left_[1-7]$/.test(name)) return 'Left arm';
    if (/^right_[1-7]$/.test(name)) return 'Right arm';
    if (name.startsWith('head_')) return 'Head';
    if (name.startsWith('left_')) return 'Left hand';
    if (name.startsWith('right_')) return 'Right hand';
    return 'Other joints';
}

/**
 * Build the joint controls from the first state snapshot.
 * @param {Array<object>} joints - Public joint descriptions and limits.
 * @returns {void} The controls are added to the document.
 */
function build(joints)
{
    const root = document.querySelector('#joint-groups');
    const groups = {};
    for (const joint of joints)
    {
        const title = group(joint.name);
        if (!groups[title])
        {
            const card = document.createElement('div');
            card.className = 'card';
            card.innerHTML = `<h2>${title}</h2>`;
            groups[title] = card;
            root.append(card);
        }
        const row = document.createElement('label');
        row.className = 'joint-row';
        row.innerHTML = `<span title="${joint.name}">${joint.name}</span><input type="range" min="${joint.minimum}" max="${joint.maximum}" step="0.01" value="${joint.position}"><input type="number" min="${joint.minimum}" max="${joint.maximum}" step="0.01" value="${joint.position.toFixed(2)}">`;
        const slider = row.children[1];
        const number = row.children[2];
        const control = {slider, number, home: joint.home, editing: false};
        controls.set(joint.name, control);
        let timer;
        /**
         * Mirror a joint edit across both inputs and queue a command.
         * @param {number|string} value - New joint position.
         * @returns {void} The command is scheduled for submission.
         */
        function send(value)
        {
            const target = Number(value);
            slider.value = target;
            number.value = target.toFixed(2);
            clearTimeout(timer);
            timer = setTimeout(() => post('/api/joints', {targets: {[joint.name]: target}}), 35);
        }
        slider.onpointerdown = () => control.editing = true;
        slider.onpointerup = slider.onpointercancel = () => control.editing = false;
        slider.oninput = () => send(slider.value);
        number.onfocus = () => control.editing = true;
        number.onblur = () => control.editing = false;
        number.onchange = () => send(number.value);
        groups[title].append(row);
    }
    built = true;
}

/**
 * Refresh untouched joint inputs from measured simulator positions.
 * @param {Array<object>} joints - Current public joint snapshots.
 * @returns {void} The visible controls are updated in place.
 */
function sync(joints)
{
    for (const joint of joints)
    {
        const control = controls.get(joint.name);
        if (control && !control.editing)
        {
            control.slider.value = joint.position;
            control.number.value = joint.position.toFixed(2);
        }
    }
}

/**
 * Populate end effector and reference frame options for the IK tester.
 * @param {object} state - Initial simulation snapshot containing link lists.
 * @returns {void} The dropdown options are initialized.
 */
function buildIk(state)
{
    const eeSelect = document.querySelector('#ik-ee');
    const frameSelect = document.querySelector('#ik-frame');
    if (!eeSelect || !frameSelect || ikBuilt) return;
    const recommendedEe = state.end_effectors || [];
    const recommendedFrames = state.frames || [];
    const allLinks = state.all_links || [];

    eeSelect.innerHTML = '';
    const recEeGroup = document.createElement('optgroup');
    recEeGroup.label = 'Recommended end effectors';
    for (const link of recommendedEe)
    {
        const opt = document.createElement('option');
        opt.value = link;
        opt.textContent = link;
        recEeGroup.append(opt);
    }
    eeSelect.append(recEeGroup);

    const otherEe = allLinks.filter(l => !recommendedEe.includes(l)).sort();
    if (otherEe.length)
    {
        const otherEeGroup = document.createElement('optgroup');
        otherEeGroup.label = 'All other links';
        for (const link of otherEe)
        {
            const opt = document.createElement('option');
            opt.value = link;
            opt.textContent = link;
            otherEeGroup.append(opt);
        }
        eeSelect.append(otherEeGroup);
    }

    if (recommendedEe.includes('left_gripper_base'))
    {
        eeSelect.value = 'left_gripper_base';
    }

    frameSelect.innerHTML = '';
    const worldOpt = document.createElement('option');
    worldOpt.value = '';
    worldOpt.textContent = 'World space (None)';
    frameSelect.append(worldOpt);

    const recFrameGroup = document.createElement('optgroup');
    recFrameGroup.label = 'Robot link frames';
    for (const link of recommendedFrames)
    {
        const opt = document.createElement('option');
        opt.value = link;
        opt.textContent = link;
        recFrameGroup.append(opt);
    }
    frameSelect.append(recFrameGroup);

    const otherFrames = allLinks.filter(l => !recommendedFrames.includes(l)).sort();
    if (otherFrames.length)
    {
        const otherFrameGroup = document.createElement('optgroup');
        otherFrameGroup.label = 'All other links';
        for (const link of otherFrames)
        {
            const opt = document.createElement('option');
            opt.value = link;
            opt.textContent = link;
            otherFrameGroup.append(opt);
        }
        frameSelect.append(otherFrameGroup);
    }

    eeSelect.onchange = () => readCurrentPose();
    frameSelect.onchange = () => readCurrentPose();
    for (const id of ['#ik-x', '#ik-y', '#ik-z'])
    {
        document.querySelector(id)?.addEventListener('input', () => { currentPosClean = false; });
    }
    for (const id of ['#ik-roll', '#ik-pitch', '#ik-yaw', '#ik-qw', '#ik-qx', '#ik-qy', '#ik-qz'])
    {
        document.querySelector(id)?.addEventListener('input', () => { currentOrientClean = false; });
    }

    const solvePosCheck = document.querySelector('#ik-solve-pos');
    const posRow = document.querySelector('#ik-pos-row');
    solvePosCheck?.addEventListener('change', () => {
        if (posRow) posRow.style.opacity = solvePosCheck.checked ? '1.0' : '0.4';
    });

    const solveOrientCheck = document.querySelector('#ik-solve-orient');
    const orientContainer = document.querySelector('#ik-orient-container');
    const orientModeSelect = document.querySelector('#ik-orient-mode');
    solveOrientCheck?.addEventListener('change', () => {
        const active = solveOrientCheck.checked;
        if (orientContainer) orientContainer.style.display = active ? 'block' : 'none';
        if (orientModeSelect) orientModeSelect.style.display = active ? 'inline-block' : 'none';
    });

    orientModeSelect?.addEventListener('change', () => {
        const mode = orientModeSelect.value;
        const eulerRow = document.querySelector('#ik-euler-row');
        const quatRow = document.querySelector('#ik-quat-row');
        if (eulerRow) eulerRow.style.display = mode === 'euler' ? 'grid' : 'none';
        if (quatRow) quatRow.style.display = mode === 'quat' ? 'grid' : 'none';
    });

    ikBuilt = true;
    readCurrentPose();
}

/**
 * Query and fill current coordinates and orientation for the chosen end effector and reference frame.
 * @returns {Promise<void>} Coordinates and orientation are placed into input fields.
 */
async function readCurrentPose()
{
    const eeSelect = document.querySelector('#ik-ee');
    const frameSelect = document.querySelector('#ik-frame');
    const statusElem = document.querySelector('#ik-status');
    if (!eeSelect || !frameSelect) return;
    const ee = eeSelect.value;
    const frame = frameSelect.value;
    if (!ee) return;
    try
    {
        const url = '/api/link-pose?link=' + encodeURIComponent(ee) + (frame ? '&frame=' + encodeURIComponent(frame) : '');
        const response = await fetch(url, {cache: 'no-store'});
        if (!response.ok) throw new Error(await response.text());
        const data = await response.json();
        currentQuat = data.quaternion;
        currentEuler = data.euler_deg;
        currentPos = data.position;
        currentPosClean = true;
        currentOrientClean = true;
        document.querySelector('#ik-x').value = data.position[0].toFixed(4);
        document.querySelector('#ik-y').value = data.position[1].toFixed(4);
        document.querySelector('#ik-z').value = data.position[2].toFixed(4);
        if (data.euler_deg)
        {
            document.querySelector('#ik-roll').value = data.euler_deg[0].toFixed(2);
            document.querySelector('#ik-pitch').value = data.euler_deg[1].toFixed(2);
            document.querySelector('#ik-yaw').value = data.euler_deg[2].toFixed(2);
        }
        if (data.quaternion)
        {
            document.querySelector('#ik-qw').value = data.quaternion[0].toFixed(4);
            document.querySelector('#ik-qx').value = data.quaternion[1].toFixed(4);
            document.querySelector('#ik-qy').value = data.quaternion[2].toFixed(4);
            document.querySelector('#ik-qz').value = data.quaternion[3].toFixed(4);
        }
        if (statusElem && !ikSolving)
        {
            const frameLabel = frame || 'world';
            const eulerStr = data.euler_deg ? ` | rot: (${data.euler_deg[0].toFixed(1)}°, ${data.euler_deg[1].toFixed(1)}°, ${data.euler_deg[2].toFixed(1)}°)` : '';
            statusElem.textContent = `Pose for ${ee} in ${frameLabel}: pos: (${data.position[0].toFixed(3)}, ${data.position[1].toFixed(3)}, ${data.position[2].toFixed(3)})${eulerStr}`;
        }
    }
    catch (error)
    {
        if (statusElem) statusElem.textContent = `Pose read failed: ${error.message}`;
    }
}

/**
 * Submit target end effector, frame, coordinates, orientation, and joint constraints for IK solving.
 * @returns {Promise<void>} The request is submitted to the simulation server.
 */
async function applyIk()
{
    const ee = document.querySelector('#ik-ee').value;
    const frame = document.querySelector('#ik-frame').value || null;
    const jointsChoice = document.querySelector('#ik-joints').value;
    const statusElem = document.querySelector('#ik-status');

    const solvePos = document.querySelector('#ik-solve-pos')?.checked;
    const solveOrient = document.querySelector('#ik-solve-orient')?.checked;

    if (!solvePos && !solveOrient)
    {
        if (statusElem) statusElem.textContent = 'Please enable at least "Solve Position" or "Solve Orientation".';
        return;
    }

    let pos = null;
    if (solvePos)
    {
        const x = Number(document.querySelector('#ik-x').value);
        const y = Number(document.querySelector('#ik-y').value);
        const z = Number(document.querySelector('#ik-z').value);
        if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z))
        {
            if (statusElem) statusElem.textContent = 'Position coordinates must be valid finite numbers.';
            return;
        }
        pos = (currentPosClean && currentPos) ? currentPos : [x, y, z];
    }

    let orient = null;
    let eulerDegrees = false;
    if (solveOrient)
    {
        const mode = document.querySelector('#ik-orient-mode')?.value || 'euler';
        if (mode === 'euler')
        {
            const r = Number(document.querySelector('#ik-roll').value);
            const p = Number(document.querySelector('#ik-pitch').value);
            const y = Number(document.querySelector('#ik-yaw').value);
            if (!Number.isFinite(r) || !Number.isFinite(p) || !Number.isFinite(y))
            {
                if (statusElem) statusElem.textContent = 'Orientation angles must be valid finite numbers.';
                return;
            }
            orient = (currentOrientClean && currentEuler) ? currentEuler : [r, p, y];
            eulerDegrees = true;
        }
        else
        {
            const qw = Number(document.querySelector('#ik-qw').value);
            const qx = Number(document.querySelector('#ik-qx').value);
            const qy = Number(document.querySelector('#ik-qy').value);
            const qz = Number(document.querySelector('#ik-qz').value);
            if (!Number.isFinite(qw) || !Number.isFinite(qx) || !Number.isFinite(qy) || !Number.isFinite(qz))
            {
                if (statusElem) statusElem.textContent = 'Quaternion coordinates must be valid finite numbers.';
                return;
            }
            orient = (currentOrientClean && currentQuat) ? currentQuat : [qw, qx, qy, qz];
            eulerDegrees = false;
        }
    }

    ikSolving = true;
    if (statusElem)
    {
        if (pos === null)
        {
            statusElem.textContent = `Solving orientation for ${ee} (keeping current position)…`;
        }
        else if (orient === null)
        {
            statusElem.textContent = `Solving position for ${ee}…`;
        }
        else
        {
            statusElem.textContent = `Solving pose for ${ee}…`;
        }
    }
    try
    {
        const response = await post('/api/ik', {
            end_effector: ee,
            position: pos,
            orientation: orient,
            frame: frame,
            joints: jointsChoice,
            euler_degrees: eulerDegrees,
        });
        if (!response.ok)
        {
            const message = await response.text();
            if (statusElem) statusElem.textContent = `IK error: ${message}`;
            ikSolving = false;
        }
    }
    catch (error)
    {
        if (statusElem) statusElem.textContent = `IK error: ${error.message}`;
        ikSolving = false;
    }
}

/**
 * Poll simulation status and camera availability.
 * @returns {Promise<void>} The dashboard reflects the latest state.
 */
async function poll()
{
    try
    {
        const state = await fetch('/api/state', {cache: 'no-store'}).then(response => response.json());
        if (!built) build(state.joints);
        else sync(state.joints);
        if (!ikBuilt && state.end_effectors) buildIk(state);
        if (state.ik_status)
        {
            document.querySelector('#ik-status').textContent = state.ik_status;
            ikSolving = false;
        }
        document.querySelector('#status').textContent = `Connected · ${state.sim_time.toFixed(2)} s`;
        cameraSources = state.camera_sources;
        for (const source of ['head', 'base'])
        {
            document.getElementById(source + '-cameras').hidden = !cameraSources.includes(source);
        }
        cameraStatus.textContent = state.camera_error || (!cameraSources.length ? 'No stereo camera pairs configured' : 'Live camera feeds · near depth bright, beyond 5 m dark');
    }
    catch (error)
    {
        document.querySelector('#status').textContent = 'Disconnected';
        cameraStatus.textContent = 'Camera feeds disconnected';
    }
}

/**
 * Fetch every head and base camera feed while the dashboard is visible.
 * @returns {Promise<void>} Available image elements receive their latest frames.
 */
async function refreshCameraFeeds()
{
    if (document.hidden || !cameraSources.length || feedsBusy) return;
    feedsBusy = true;
    try
    {
        const names = cameraSources.flatMap(source => ['left-rgb', 'right-rgb', 'depth'].map(feed => source + '-' + feed));
        await Promise.all(names.map(async name =>
        {
            const response = await fetch('/api/camera/' + name, {cache: 'no-store'});
            if (!response.ok) return;
            const blob = await response.blob();
            const image = document.getElementById(name);
            const previous = image.dataset.objectUrl;
            const url = URL.createObjectURL(blob);
            image.src = url;
            image.dataset.objectUrl = url;
            if (previous) URL.revokeObjectURL(previous);
        }));
    }
    catch (error)
    {
        cameraStatus.textContent = 'Waiting for camera frames…';
    }
    finally
    {
        feedsBusy = false;
    }
}

/**
 * Submit a base command using the currently selected speed limits.
 * @param {string} kind - Forward, back, left, right, or stop.
 * @returns {void} The selected drive command is submitted.
 */
function command(kind)
{
    drive = kind;
    const linear = Number(document.querySelector('#linear').value);
    const angular = Number(document.querySelector('#angular').value);
    const options = {forward: [linear, 0], back: [-linear, 0], left: [0, angular], right: [0, -angular], stop: [0, 0]};
    post('/api/drive', {linear: options[kind][0], angular: options[kind][1]});
}

/**
 * Restore every visible joint control and queue the configured home pose.
 * @returns {void} The home request is submitted.
 */
function resetHome()
{
    for (const control of controls.values())
    {
        control.editing = false;
        control.slider.value = control.home;
        control.number.value = control.home.toFixed(2);
    }
    post('/api/home', {});
}

setInterval(poll, 250);
poll();
setInterval(refreshCameraFeeds, 500);
for (const id of ['linear', 'angular'])
{
    const input = document.querySelector('#' + id);
    input.oninput = () => input.nextElementSibling.value = Number(input.value).toFixed(2);
}
for (const button of document.querySelectorAll('[data-drive]'))
{
    button.onpointerdown = event =>
    {
        event.preventDefault();
        command(button.dataset.drive);
    };
    button.onpointerup = button.onpointercancel = button.onpointerleave = () =>
    {
        if (drive === button.dataset.drive) command('stop');
    };
}
setInterval(() =>
{
    if (drive !== 'stop') command(drive);
}, 100);
const keys = {w: 'forward', s: 'back', a: 'left', d: 'right'};
window.onkeydown = event =>
{
    if (event.repeat || !keys[event.key.toLowerCase()] || /INPUT/.test(event.target.tagName)) return;
    command(keys[event.key.toLowerCase()]);
};
window.onkeyup = event =>
{
    if (keys[event.key.toLowerCase()] === drive) command('stop');
};
window.onblur = () => command('stop');
document.querySelector('#home').onclick = resetHome;
document.querySelector('#ik-read').onclick = readCurrentPose;
document.querySelector('#ik-solve').onclick = applyIk;
"""
