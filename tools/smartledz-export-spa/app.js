"use strict";

const ID_PATTERN = /^[A-Za-z_][A-Za-z0-9_]*$/;
const MAC_PATTERN = /^([0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2}$/;
const TARGET_PATTERN = /^0x[0-9A-Fa-f]{1,4}$/;
const VENDOR_PATTERN = /^(0x[0-9A-Fa-f]{1,4}|\d{1,5})$/;
const COMMIT_SHA_PATTERN = /^[0-9A-Fa-f]{40}$/;
const CT_DUV_MIN = -6;
const CT_DUV_MAX = 6;
const CT_DUV_STEP = "0.1";
const DEVICE_TYPE_ALLOWED = new Set(["dimmable", "tunable", "synca"]);
const SOURCE_REPO_OWNER = "hrko";
const SOURCE_REPO_NAME = "esphome-smart-ledz";
const SOURCE_REPO_URL = `https://github.com/${SOURCE_REPO_OWNER}/${SOURCE_REPO_NAME}.git`;

const TYPE_CODE_TO_DEVICE_TYPE = new Map([
  [1, "dimmable"],
  [53, "dimmable"],
  [6, "dimmable"],
  [8, "dimmable"],
  [7, "dimmable"],
  [2, "tunable"],
  [54, "tunable"],
  [52, "synca"],
  [48, "synca"],
  [55, "synca"],
]);

const state = {
  parsed: false,
  fileName: "",
  meta: {
    meshName: "",
    meshPassword: "",
  },
  devices: [],
  groups: [],
  lightConfigs: new Map(),
  ble: {
    id: "ledz_node",
    selectedDeviceKey: "",
    macAddress: "",
    autoConnect: true,
  },
  hub: {
    id: "ledz_hub",
    vendorId: "0x0211",
    pollInterval: "2s",
    txInterval: "120ms",
    powerOnSettle: "400ms",
    secretMeshNameKey: "smart_ledz_mesh_name",
    secretMeshPasswordKey: "smart_ledz_mesh_password",
    extraYaml: "",
  },
  source: {
    ref: "main",
    latestRelease: null,
    statusText: "Default ref is main.",
    statusTone: "status-muted",
    fetching: false,
  },
};

const refs = {};

document.addEventListener("DOMContentLoaded", () => {
  bindRefs();
  bindEvents();
  refreshAll();
});

function bindRefs() {
  refs.fileInput = document.getElementById("export-file");
  refs.inputStatus = document.getElementById("input-status");
  refs.meshSummary = document.getElementById("mesh-summary");

  refs.bleId = document.getElementById("ble-id");
  refs.bleDeviceSelect = document.getElementById("ble-device-select");
  refs.bleMacAddress = document.getElementById("ble-mac-address");
  refs.bleAutoConnect = document.getElementById("ble-auto-connect");

  refs.hubId = document.getElementById("hub-id");
  refs.hubVendorId = document.getElementById("hub-vendor-id");
  refs.hubPollInterval = document.getElementById("hub-poll-interval");
  refs.hubTxInterval = document.getElementById("hub-tx-interval");
  refs.hubPowerOnSettle = document.getElementById("hub-power-on-settle");
  refs.componentRef = document.getElementById("component-ref");
  refs.fetchLatestReleaseRef = document.getElementById("fetch-latest-release-ref");
  refs.releaseRefStatus = document.getElementById("release-ref-status");
  refs.secretMeshNameKey = document.getElementById("secret-mesh-name-key");
  refs.secretMeshPasswordKey = document.getElementById("secret-mesh-password-key");
  refs.hubExtraYaml = document.getElementById("hub-extra-yaml");

  refs.deviceCount = document.getElementById("device-count");
  refs.groupCount = document.getElementById("group-count");
  refs.deviceList = document.getElementById("device-list");
  refs.groupList = document.getElementById("group-list");

  refs.selectAllDevices = document.getElementById("select-all-devices");
  refs.clearDevices = document.getElementById("clear-devices");
  refs.selectAllGroups = document.getElementById("select-all-groups");
  refs.clearGroups = document.getElementById("clear-groups");

  refs.lightEditorList = document.getElementById("light-editor-list");

  refs.validationMessages = document.getElementById("validation-messages");
  refs.yamlOutput = document.getElementById("yaml-output");
  refs.secretsOutput = document.getElementById("secrets-output");
  refs.copyYaml = document.getElementById("copy-yaml");
  refs.copySecrets = document.getElementById("copy-secrets");
}

function bindEvents() {
  refs.fileInput.addEventListener("change", onFileSelected);

  refs.bleId.addEventListener("input", () => {
    state.ble.id = refs.bleId.value.trim();
    state.hub.bleClientId = state.ble.id;
    updateOutputs();
  });

  refs.bleDeviceSelect.addEventListener("change", () => {
    state.ble.selectedDeviceKey = refs.bleDeviceSelect.value;
    const selected = findDeviceByKey(state.ble.selectedDeviceKey);
    if (selected && selected.mac) {
      state.ble.macAddress = normalizeMac(selected.mac);
      refs.bleMacAddress.value = state.ble.macAddress;
    }
    updateOutputs();
  });

  refs.bleMacAddress.addEventListener("input", () => {
    state.ble.macAddress = refs.bleMacAddress.value.trim().toUpperCase();
    updateOutputs();
  });

  refs.bleAutoConnect.addEventListener("change", () => {
    state.ble.autoConnect = refs.bleAutoConnect.checked;
    updateOutputs();
  });

  refs.hubId.addEventListener("input", () => {
    state.hub.id = refs.hubId.value.trim();
    updateOutputs();
  });

  refs.hubVendorId.addEventListener("input", () => {
    state.hub.vendorId = refs.hubVendorId.value.trim();
    updateOutputs();
  });

  refs.hubPollInterval.addEventListener("input", () => {
    state.hub.pollInterval = refs.hubPollInterval.value.trim();
    updateOutputs();
  });

  refs.hubTxInterval.addEventListener("input", () => {
    state.hub.txInterval = refs.hubTxInterval.value.trim();
    updateOutputs();
  });

  refs.hubPowerOnSettle.addEventListener("input", () => {
    state.hub.powerOnSettle = refs.hubPowerOnSettle.value.trim();
    updateOutputs();
  });

  refs.componentRef.addEventListener("input", () => {
    state.source.ref = refs.componentRef.value.trim();
    if (state.source.latestRelease && state.source.latestRelease.commitSha !== state.source.ref) {
      state.source.latestRelease = null;
      setSourceStatus("Manual ref mode.", "status-muted");
    }
    updateOutputs();
  });

  refs.fetchLatestReleaseRef.addEventListener("click", onFetchLatestReleaseRef);

  refs.secretMeshNameKey.addEventListener("input", () => {
    state.hub.secretMeshNameKey = refs.secretMeshNameKey.value.trim();
    updateOutputs();
  });

  refs.secretMeshPasswordKey.addEventListener("input", () => {
    state.hub.secretMeshPasswordKey = refs.secretMeshPasswordKey.value.trim();
    updateOutputs();
  });

  refs.hubExtraYaml.addEventListener("input", () => {
    state.hub.extraYaml = refs.hubExtraYaml.value;
    updateOutputs();
  });

  refs.selectAllDevices.addEventListener("click", () => {
    setAllSelected(state.devices, true);
    renderTargetLists();
    renderLightEditors();
    updateOutputs();
  });

  refs.clearDevices.addEventListener("click", () => {
    setAllSelected(state.devices, false);
    renderTargetLists();
    renderLightEditors();
    updateOutputs();
  });

  refs.selectAllGroups.addEventListener("click", () => {
    setAllSelected(state.groups, true);
    renderTargetLists();
    renderLightEditors();
    updateOutputs();
  });

  refs.clearGroups.addEventListener("click", () => {
    setAllSelected(state.groups, false);
    renderTargetLists();
    renderLightEditors();
    updateOutputs();
  });

  refs.deviceList.addEventListener("change", onTargetToggle);
  refs.groupList.addEventListener("change", onTargetToggle);

  refs.lightEditorList.addEventListener("input", onLightFieldUpdate);
  refs.lightEditorList.addEventListener("change", onLightFieldUpdate);

  refs.copyYaml.addEventListener("click", async () => {
    if (refs.copyYaml.disabled) {
      return;
    }
    const ok = await copyText(refs.yamlOutput.textContent || "");
    flashCopyState(refs.copyYaml, ok ? "Copied" : "Copy failed");
  });

  refs.copySecrets.addEventListener("click", async () => {
    if (refs.copySecrets.disabled) {
      return;
    }
    const ok = await copyText(refs.secretsOutput.textContent || "");
    flashCopyState(refs.copySecrets, ok ? "Copied" : "Copy failed");
  });
}

function refreshAll() {
  syncFormFromState();
  renderInputSummary();
  renderBleSelect();
  renderTargetLists();
  renderLightEditors();
  updateOutputs();
}

function syncFormFromState() {
  refs.bleId.value = state.ble.id;
  refs.bleMacAddress.value = state.ble.macAddress;
  refs.bleAutoConnect.checked = state.ble.autoConnect;

  refs.hubId.value = state.hub.id;
  refs.hubVendorId.value = state.hub.vendorId;
  refs.hubPollInterval.value = state.hub.pollInterval;
  refs.hubTxInterval.value = state.hub.txInterval;
  refs.hubPowerOnSettle.value = state.hub.powerOnSettle;
  refs.componentRef.value = state.source.ref;
  refs.fetchLatestReleaseRef.disabled = state.source.fetching;
  renderSourceStatus();
  refs.secretMeshNameKey.value = state.hub.secretMeshNameKey;
  refs.secretMeshPasswordKey.value = state.hub.secretMeshPasswordKey;
  refs.hubExtraYaml.value = state.hub.extraYaml;
}

async function onFileSelected(event) {
  const file = event.target.files && event.target.files[0];
  if (!file) {
    return;
  }

  state.fileName = file.name;
  refs.inputStatus.textContent = `Reading ${file.name}...`;

  try {
    const text = await file.text();
    const raw = JSON.parse(text);
    const parsed = parseRawExport(raw);

    state.parsed = true;
    state.meta = parsed.meta;
    state.devices = parsed.devices;
    state.groups = parsed.groups;
    state.lightConfigs.clear();

    const targets = getAllTargets();
    for (const target of targets) {
      state.lightConfigs.set(target.key, buildDefaultLightConfig(target));
    }

    if (state.devices.length > 0) {
      state.ble.selectedDeviceKey = state.devices[0].key;
      state.ble.macAddress = normalizeMac(state.devices[0].mac || "");
    } else {
      state.ble.selectedDeviceKey = "";
      state.ble.macAddress = "";
    }

    refs.inputStatus.textContent = `Loaded ${file.name}`;
    refreshAll();
  } catch (error) {
    state.parsed = false;
    state.devices = [];
    state.groups = [];
    state.lightConfigs.clear();
    refs.inputStatus.textContent = `Failed to read ${file.name}: ${error.message}`;
    refreshAll();
  }
}

async function onFetchLatestReleaseRef() {
  if (state.source.fetching) {
    return;
  }

  state.source.fetching = true;
  refs.fetchLatestReleaseRef.disabled = true;
  setSourceStatus("Fetching latest release commit...", "status-muted");

  try {
    const latest = await fetchLatestReleaseCommitHash();
    state.source.ref = latest.commitSha;
    state.source.latestRelease = latest;
    refs.componentRef.value = state.source.ref;
    setSourceStatus(`Loaded ${latest.tagName} (${latest.commitSha.slice(0, 12)}...)`, "status-ok");
  } catch (error) {
    state.source.latestRelease = null;
    setSourceStatus(`Failed to fetch latest release: ${error.message}`, "status-error");
  } finally {
    state.source.fetching = false;
    refs.fetchLatestReleaseRef.disabled = false;
    updateOutputs();
  }
}

async function fetchLatestReleaseCommitHash() {
  const latestReleaseUrl = `https://api.github.com/repos/${SOURCE_REPO_OWNER}/${SOURCE_REPO_NAME}/releases/latest`;
  const release = await fetchJson(latestReleaseUrl);
  const tagName = asString(release.tag_name);

  if (!tagName) {
    throw new Error("Latest release tag was not found.");
  }

  const commitUrl = `https://api.github.com/repos/${SOURCE_REPO_OWNER}/${SOURCE_REPO_NAME}/commits/${encodeURIComponent(tagName)}`;
  const commit = await fetchJson(commitUrl);
  const commitSha = asString(commit.sha);

  if (!COMMIT_SHA_PATTERN.test(commitSha)) {
    throw new Error("Latest release commit hash is invalid.");
  }

  return {
    tagName,
    commitSha,
  };
}

async function fetchJson(url) {
  const response = await fetch(url, {
    headers: {
      Accept: "application/vnd.github+json",
    },
  });

  if (!response.ok) {
    let detail = "";
    try {
      const body = await response.json();
      detail = asString(body && body.message);
    } catch (error) {
      detail = "";
    }
    if (detail) {
      throw new Error(`HTTP ${response.status}: ${detail}`);
    }
    throw new Error(`HTTP ${response.status}`);
  }

  return response.json();
}

function setSourceStatus(text, tone) {
  state.source.statusText = text;
  state.source.statusTone = tone;
  renderSourceStatus();
}

function renderSourceStatus() {
  refs.releaseRefStatus.textContent = state.source.statusText;
  refs.releaseRefStatus.className = `muted release-ref-status ${state.source.statusTone}`;
}

function parseRawExport(raw) {
  if (!raw || typeof raw !== "object") {
    throw new Error("JSON root must be an object");
  }

  const meta = {
    meshName: asString(raw.meshName),
    meshPassword: asString(raw.meshPassword),
  };

  const devices = parseDevices(raw);
  const groups = parseGroups(raw);

  return { meta, devices, groups };
}

function parseDevices(raw) {
  const macByAddress = new Map();

  if (Array.isArray(raw.deviceAddrArr) && Array.isArray(raw.deviceMac)) {
    raw.deviceAddrArr.forEach((address, index) => {
      const addressInt = toInt(address);
      if (addressInt === null) {
        return;
      }
      const macValue = index < raw.deviceMac.length ? asString(raw.deviceMac[index]) : "";
      if (macValue) {
        macByAddress.set(addressInt, normalizeMac(macValue));
      }
    });
  }

  const rows = Array.isArray(raw.deviceArr) ? raw.deviceArr : [];
  const devices = [];

  for (const row of rows) {
    if (!Array.isArray(row) || row.length === 0) {
      continue;
    }
    const meshAddress = toInt(row[0]);
    if (meshAddress === null) {
      continue;
    }
    const typeCode = toInt(row[1]);
    const name = asString(row[2]) || `Device ${meshAddress}`;
    const mac = macByAddress.get(meshAddress) || "";

    devices.push({
      key: `device:${meshAddress}`,
      kind: "device",
      meshAddress,
      typeCode,
      label: name,
      mac,
      targetHex: toHex16(meshAddress),
      selected: true,
    });
  }

  devices.sort((a, b) => a.meshAddress - b.meshAddress);
  return devices;
}

function parseGroups(raw) {
  const groupsByLow = new Map();

  const groupRows = Array.isArray(raw.groupArr) ? raw.groupArr : [];
  for (const row of groupRows) {
    if (!Array.isArray(row) || row.length === 0) {
      continue;
    }
    const lowByte = toInt(row[0]);
    if (lowByte === null) {
      continue;
    }
    groupsByLow.set(lowByte, {
      key: `group:${lowByte}`,
      kind: "group",
      lowByte,
      typeCode: toInt(row[1]),
      label: asString(row[2]) || `Group ${lowByte}`,
      targetHex: toHex16(0x8000 | (lowByte & 0xff)),
      memberCount: 0,
      selected: true,
    });
  }

  const groupStateRows = Array.isArray(raw.group_p_Arr) ? raw.group_p_Arr : [];
  for (const row of groupStateRows) {
    if (!Array.isArray(row) || row.length === 0) {
      continue;
    }
    const lowByte = toInt(row[0]);
    if (lowByte === null) {
      continue;
    }
    const existing = groupsByLow.get(lowByte);
    const members = Array.isArray(row[4]) ? row[4] : [];

    if (existing) {
      existing.memberCount = members.length;
      if (existing.typeCode === null) {
        existing.typeCode = toInt(row[1]);
      }
      continue;
    }

    groupsByLow.set(lowByte, {
      key: `group:${lowByte}`,
      kind: "group",
      lowByte,
      typeCode: toInt(row[1]),
      label: `Group ${lowByte}`,
      targetHex: toHex16(0x8000 | (lowByte & 0xff)),
      memberCount: members.length,
      selected: true,
    });
  }

  const groups = Array.from(groupsByLow.values());
  groups.sort((a, b) => a.lowByte - b.lowByte);
  return groups;
}

function buildDefaultLightConfig(target) {
  return {
    id: `ledz_light_${target.targetHex.toLowerCase()}`,
    name: target.label,
    target: target.targetHex,
    deviceType: inferDeviceType(target.typeCode),
    ctDuv: "0",
    ignoreTransition: true,
    extraYaml: "",
  };
}

function inferDeviceType(typeCode) {
  if (typeCode !== null && TYPE_CODE_TO_DEVICE_TYPE.has(typeCode)) {
    return TYPE_CODE_TO_DEVICE_TYPE.get(typeCode);
  }
  return "dimmable";
}

function renderInputSummary() {
  refs.meshSummary.innerHTML = "";

  if (!state.parsed) {
    return;
  }

  const chips = [
    { label: "Mesh Name", value: state.meta.meshName || "(missing)" },
    { label: "Devices", value: String(state.devices.length) },
    { label: "Groups", value: String(state.groups.length) },
  ];

  for (const chip of chips) {
    const div = document.createElement("div");
    div.className = "summary-chip";
    div.innerHTML = `<small>${escapeHtml(chip.label)}</small><strong>${escapeHtml(chip.value)}</strong>`;
    refs.meshSummary.appendChild(div);
  }
}

function renderBleSelect() {
  refs.bleDeviceSelect.innerHTML = "";

  if (state.devices.length === 0) {
    const option = document.createElement("option");
    option.value = "";
    option.textContent = "No device options";
    refs.bleDeviceSelect.appendChild(option);
    refs.bleDeviceSelect.disabled = true;
    return;
  }

  refs.bleDeviceSelect.disabled = false;

  for (const device of state.devices) {
    const option = document.createElement("option");
    option.value = device.key;
    option.textContent = `${device.label} (${device.mac || "MAC unknown"})`;
    refs.bleDeviceSelect.appendChild(option);
  }

  if (!findDeviceByKey(state.ble.selectedDeviceKey)) {
    state.ble.selectedDeviceKey = state.devices[0].key;
    if (!state.ble.macAddress) {
      state.ble.macAddress = normalizeMac(state.devices[0].mac || "");
      refs.bleMacAddress.value = state.ble.macAddress;
    }
  }

  refs.bleDeviceSelect.value = state.ble.selectedDeviceKey;
}

function renderTargetLists() {
  refs.deviceCount.textContent = String(state.devices.length);
  refs.groupCount.textContent = String(state.groups.length);

  renderTargetList(refs.deviceList, state.devices, (item) => {
    const macText = item.mac ? `MAC ${item.mac}` : "MAC unknown";
    return `Addr ${item.targetHex} / type ${nullableTypeCode(item.typeCode)} / ${macText}`;
  });

  renderTargetList(refs.groupList, state.groups, (item) => {
    return `Group low ${item.lowByte} / target ${item.targetHex} / members ${item.memberCount}`;
  });
}

function renderTargetList(container, items, getSubText) {
  container.innerHTML = "";

  if (items.length === 0) {
    const li = document.createElement("li");
    li.className = "placeholder";
    li.textContent = "No entries";
    container.appendChild(li);
    return;
  }

  for (const item of items) {
    const li = document.createElement("li");
    li.innerHTML = `
      <label class="target-label">
        <input type="checkbox" data-role="target-toggle" data-key="${escapeHtml(item.key)}" ${item.selected ? "checked" : ""} />
        <span class="target-meta">
          <span class="target-title">${escapeHtml(item.label)}</span>
          <span class="target-sub">${escapeHtml(getSubText(item))}</span>
        </span>
      </label>
    `;
    container.appendChild(li);
  }
}

function renderLightEditors() {
  const selectedTargets = getSelectedTargets();
  refs.lightEditorList.innerHTML = "";

  if (selectedTargets.length === 0) {
    const placeholder = document.createElement("div");
    placeholder.className = "placeholder";
    placeholder.textContent = "Select at least one device or group to configure light output.";
    refs.lightEditorList.appendChild(placeholder);
    return;
  }

  for (const target of selectedTargets) {
    const cfg = ensureLightConfig(target);
    const card = document.createElement("article");
    card.className = "light-card";

    const deviceTypeOptions = ["dimmable", "tunable", "synca"]
      .map((option) => {
        const selected = option === cfg.deviceType ? "selected" : "";
        return `<option value="${option}" ${selected}>${option}</option>`;
      })
      .join("");

    const sourceText =
      target.kind === "device"
        ? `device target ${target.targetHex} / type ${nullableTypeCode(target.typeCode)}`
        : `group target ${target.targetHex} (low ${target.lowByte}) / type ${nullableTypeCode(target.typeCode)}`;

    card.innerHTML = `
      <div class="light-card-header">
        <div>
          <h3>${escapeHtml(target.label)}</h3>
          <p class="muted">${escapeHtml(sourceText)}</p>
        </div>
        <span class="light-badge">${target.kind}</span>
      </div>
      <div class="light-grid-fields light-grid-basic-fields">
        <label class="field">
          <span>name</span>
          <input data-role="light-name" data-key="${escapeHtml(target.key)}" type="text" value="${escapeHtml(cfg.name)}" />
        </label>
      </div>
      <details class="advanced-block">
        <summary>Advanced Light Settings</summary>
        <div class="light-grid-fields">
          <label class="field">
            <span>id</span>
            <input data-role="light-id" data-key="${escapeHtml(target.key)}" type="text" value="${escapeHtml(cfg.id)}" />
          </label>
          <label class="field">
            <span>target</span>
            <input data-role="light-target" data-key="${escapeHtml(target.key)}" type="text" value="${escapeHtml(cfg.target)}" />
          </label>
          <label class="field">
            <span>device_type</span>
            <select data-role="light-device-type" data-key="${escapeHtml(target.key)}">${deviceTypeOptions}</select>
          </label>
          <label class="field">
            <span>ct_duv</span>
            <input
              data-role="light-ct-duv"
              data-key="${escapeHtml(target.key)}"
              type="number"
              min="${CT_DUV_MIN}"
              max="${CT_DUV_MAX}"
              step="${CT_DUV_STEP}"
              value="${escapeHtml(cfg.ctDuv)}"
            />
          </label>
          <label class="check-field">
            <input data-role="light-ignore-transition" data-key="${escapeHtml(target.key)}" type="checkbox" ${
              cfg.ignoreTransition ? "checked" : ""
            } />
            <span>ignore_transition</span>
          </label>
        </div>
        <label class="field">
          <span>Light Extra YAML Lines</span>
          <textarea data-role="light-extra" data-key="${escapeHtml(target.key)}" rows="3" placeholder="example:\ndefault_transition_length: 0s">${escapeHtml(
            cfg.extraYaml
          )}</textarea>
        </label>
      </details>
    `;

    refs.lightEditorList.appendChild(card);
  }
}

function updateOutputs() {
  const errors = [];
  const warnings = [];

  if (!state.parsed) {
    errors.push("Load a SmartLEDZ raw export JSON file.");
  }

  if (state.parsed) {
    if (!state.meta.meshName) {
      errors.push("meshName is missing in export JSON.");
    }
    if (!state.meta.meshPassword) {
      errors.push("meshPassword is missing in export JSON.");
    }
  }

  const selectedTargets = getSelectedTargets();
  if (state.parsed && selectedTargets.length === 0) {
    errors.push("Select at least one device or group.");
  }

  validateIdentifier("ble_client.id", state.ble.id, errors);
  validateIdentifier("smart_ledz.id", state.hub.id, errors);
  validateIdentifier("secret mesh_name key", state.hub.secretMeshNameKey, errors);
  validateIdentifier("secret mesh_password key", state.hub.secretMeshPasswordKey, errors);

  if (!state.ble.macAddress) {
    errors.push("ble_client.mac_address is required.");
  } else if (!MAC_PATTERN.test(state.ble.macAddress)) {
    errors.push(`ble_client.mac_address is invalid: ${state.ble.macAddress}`);
  }

  if (!state.hub.vendorId || !VENDOR_PATTERN.test(state.hub.vendorId)) {
    errors.push("smart_ledz.vendor_id must be a hex value like 0x0211 or a decimal number.");
  }

  if (!state.source.ref) {
    errors.push("external_components.source.ref is required.");
  }

  if (!state.hub.pollInterval) {
    errors.push("smart_ledz.poll_interval is required.");
  }
  if (!state.hub.txInterval) {
    errors.push("smart_ledz.tx_interval is required.");
  }
  if (!state.hub.powerOnSettle) {
    errors.push("smart_ledz.power_on_settle is required.");
  }

  const idSet = new Set();
  let usesExtraYaml = Boolean(state.hub.extraYaml.trim());

  for (const target of selectedTargets) {
    const cfg = ensureLightConfig(target);

    if (!ID_PATTERN.test(cfg.id)) {
      errors.push(`Invalid light id for ${target.label}: ${cfg.id}`);
    }
    if (idSet.has(cfg.id)) {
      errors.push(`Duplicate light id: ${cfg.id}`);
    }
    idSet.add(cfg.id);

    if (!TARGET_PATTERN.test(cfg.target)) {
      errors.push(`Invalid target for ${target.label}: ${cfg.target}`);
    }

    if (!DEVICE_TYPE_ALLOWED.has(cfg.deviceType)) {
      errors.push(`Invalid device_type for ${target.label}: ${cfg.deviceType}`);
    }

    const ctDuv = parseCtDuv(cfg.ctDuv);
    if (ctDuv === null || ctDuv < CT_DUV_MIN || ctDuv > CT_DUV_MAX) {
      errors.push(`Invalid ct_duv for ${target.label}: ${cfg.ctDuv}`);
    }

    if (cfg.extraYaml.trim()) {
      usesExtraYaml = true;
    }
  }

  if (usesExtraYaml) {
    warnings.push("Extra YAML lines are injected as-is. Verify YAML syntax before use.");
  }

  renderMessages(errors, warnings);

  if (errors.length > 0) {
    refs.yamlOutput.textContent = "# Fix validation errors to generate YAML.";
    refs.secretsOutput.textContent = "# Fix validation errors to generate secrets output.";
    refs.copyYaml.disabled = true;
    refs.copySecrets.disabled = true;
    return;
  }

  const yaml = generateEspHomeYaml(selectedTargets);
  const secrets = generateSecretsYaml();

  refs.yamlOutput.textContent = yaml;
  refs.secretsOutput.textContent = secrets;
  refs.copyYaml.disabled = false;
  refs.copySecrets.disabled = false;
}

function renderMessages(errors, warnings) {
  refs.validationMessages.innerHTML = "";

  if (errors.length === 0 && warnings.length === 0) {
    return;
  }

  if (errors.length > 0) {
    refs.validationMessages.appendChild(createMessageBox("error", "Errors", errors));
  }

  if (warnings.length > 0) {
    refs.validationMessages.appendChild(createMessageBox("warn", "Warnings", warnings));
  }
}

function createMessageBox(kind, title, lines) {
  const box = document.createElement("div");
  box.className = `message-box ${kind}`;

  const safeLines = lines.map((line) => `<li>${escapeHtml(line)}</li>`).join("");
  box.innerHTML = `<strong>${escapeHtml(title)}</strong><ul>${safeLines}</ul>`;
  return box;
}

function generateEspHomeYaml(selectedTargets) {
  const bleId = state.ble.id;
  const hubId = state.hub.id;
  const macAddress = normalizeMac(state.ble.macAddress);

  const lines = [];
  lines.push("external_components:");
  lines.push("  - source:");
  lines.push("      type: git");
  lines.push("      url: " + SOURCE_REPO_URL);
  lines.push(buildRefYamlLine());
  lines.push("    components: [smart_ledz]");
  lines.push("");

  lines.push("ble_client:");
  lines.push("  - id: " + bleId);
  lines.push("    mac_address: " + yamlQuote(macAddress));
  lines.push("    auto_connect: " + boolYaml(state.ble.autoConnect));
  lines.push("");

  lines.push("smart_ledz:");
  lines.push("  - id: " + hubId);
  lines.push("    ble_client_id: " + bleId);
  lines.push("    mesh_name: !secret " + state.hub.secretMeshNameKey);
  lines.push("    mesh_password: !secret " + state.hub.secretMeshPasswordKey);
  lines.push("    vendor_id: " + state.hub.vendorId);
  lines.push("    poll_interval: " + state.hub.pollInterval);
  lines.push("    tx_interval: " + state.hub.txInterval);
  lines.push("    power_on_settle: " + state.hub.powerOnSettle);
  appendExtraYaml(lines, state.hub.extraYaml, 4);
  lines.push("");

  lines.push("light:");
  for (const target of selectedTargets) {
    const cfg = ensureLightConfig(target);
    const normalizedTarget = normalizeTarget(cfg.target);

    lines.push("  - platform: smart_ledz");
    lines.push("    id: " + cfg.id);
    lines.push("    name: " + yamlQuote(cfg.name));
    lines.push("    smart_ledz_id: " + hubId);
    lines.push("    target: " + normalizedTarget);
    lines.push("    device_type: " + cfg.deviceType);
    lines.push("    ct_duv: " + normalizeCtDuv(cfg.ctDuv));
    lines.push("    ignore_transition: " + boolYaml(cfg.ignoreTransition));
    appendExtraYaml(lines, cfg.extraYaml, 4);
  }

  return lines.join("\n") + "\n";
}

function buildRefYamlLine() {
  const base = "      ref: " + yamlQuote(state.source.ref);
  if (!state.source.latestRelease) {
    return base;
  }
  if (state.source.latestRelease.commitSha !== state.source.ref) {
    return base;
  }
  const tagName = asString(state.source.latestRelease.tagName);
  if (!tagName) {
    return base;
  }
  return `${base} # ${tagName}`;
}

function generateSecretsYaml() {
  const lines = [];
  lines.push(`${state.hub.secretMeshNameKey}: ${yamlQuote(state.meta.meshName)}`);
  lines.push(`${state.hub.secretMeshPasswordKey}: ${yamlQuote(state.meta.meshPassword)}`);
  return lines.join("\n") + "\n";
}

function appendExtraYaml(lines, rawText, indentSpaces) {
  if (!rawText.trim()) {
    return;
  }
  const indent = " ".repeat(indentSpaces);
  rawText
    .split(/\r?\n/)
    .map((line) => line.trim())
    .filter((line) => line.length > 0)
    .forEach((line) => lines.push(indent + line));
}

function onTargetToggle(event) {
  const input = event.target;
  if (!(input instanceof HTMLInputElement)) {
    return;
  }
  if (input.dataset.role !== "target-toggle") {
    return;
  }
  const key = input.dataset.key;
  const target = findTargetByKey(key);
  if (!target) {
    return;
  }

  target.selected = input.checked;
  renderLightEditors();
  updateOutputs();
}

function onLightFieldUpdate(event) {
  const field = event.target;
  if (!(field instanceof HTMLElement)) {
    return;
  }

  const role = field.dataset.role;
  const key = field.dataset.key;
  if (!role || !key) {
    return;
  }

  const cfg = state.lightConfigs.get(key);
  if (!cfg) {
    return;
  }

  if (role === "light-name" && field instanceof HTMLInputElement) {
    cfg.name = field.value;
  } else if (role === "light-id" && field instanceof HTMLInputElement) {
    cfg.id = field.value.trim();
  } else if (role === "light-target" && field instanceof HTMLInputElement) {
    cfg.target = field.value.trim();
  } else if (role === "light-device-type" && field instanceof HTMLSelectElement) {
    cfg.deviceType = field.value;
  } else if (role === "light-ct-duv" && field instanceof HTMLInputElement) {
    cfg.ctDuv = field.value.trim();
  } else if (role === "light-ignore-transition" && field instanceof HTMLInputElement) {
    cfg.ignoreTransition = field.checked;
  } else if (role === "light-extra" && field instanceof HTMLTextAreaElement) {
    cfg.extraYaml = field.value;
  } else {
    return;
  }

  updateOutputs();
}

function findTargetByKey(key) {
  for (const item of getAllTargets()) {
    if (item.key === key) {
      return item;
    }
  }
  return null;
}

function findDeviceByKey(key) {
  return state.devices.find((device) => device.key === key) || null;
}

function ensureLightConfig(target) {
  let cfg = state.lightConfigs.get(target.key);
  if (!cfg) {
    cfg = buildDefaultLightConfig(target);
    state.lightConfigs.set(target.key, cfg);
  }
  return cfg;
}

function getAllTargets() {
  return [...state.devices, ...state.groups];
}

function getSelectedTargets() {
  const selected = [];
  for (const target of [...state.devices, ...state.groups]) {
    if (target.selected) {
      selected.push(target);
    }
  }
  return selected;
}

function setAllSelected(items, value) {
  for (const item of items) {
    item.selected = value;
  }
}

function normalizeTarget(value) {
  if (!TARGET_PATTERN.test(value)) {
    return value;
  }
  const numeric = Number.parseInt(value.slice(2), 16);
  if (!Number.isFinite(numeric) || numeric < 0 || numeric > 0xffff) {
    return value;
  }
  return toHex16(numeric);
}

function parseCtDuv(value) {
  const parsed = Number.parseFloat(String(value).trim());
  if (!Number.isFinite(parsed)) {
    return null;
  }
  return parsed;
}

function normalizeCtDuv(value) {
  const parsed = parseCtDuv(value);
  if (parsed === null) {
    return String(value).trim();
  }
  return Number.isInteger(parsed) ? String(parsed) : String(parsed);
}

function validateIdentifier(name, value, errors) {
  if (!value) {
    errors.push(`${name} is required.`);
    return;
  }
  if (!ID_PATTERN.test(value)) {
    errors.push(`${name} must match ${ID_PATTERN.toString()}`);
  }
}

function copyText(text) {
  if (!text) {
    return Promise.resolve(false);
  }

  if (navigator.clipboard && navigator.clipboard.writeText) {
    return navigator.clipboard
      .writeText(text)
      .then(() => true)
      .catch(() => fallbackCopy(text));
  }

  return Promise.resolve(fallbackCopy(text));
}

function fallbackCopy(text) {
  try {
    const textarea = document.createElement("textarea");
    textarea.value = text;
    textarea.style.position = "fixed";
    textarea.style.opacity = "0";
    document.body.appendChild(textarea);
    textarea.focus();
    textarea.select();
    const ok = document.execCommand("copy");
    document.body.removeChild(textarea);
    return ok;
  } catch (error) {
    return false;
  }
}

function flashCopyState(button, label) {
  const original = button.textContent;
  button.textContent = label;
  window.setTimeout(() => {
    button.textContent = original;
  }, 1200);
}

function toInt(value) {
  const parsed = Number.parseInt(value, 10);
  return Number.isFinite(parsed) ? parsed : null;
}

function toHex16(value) {
  const num = Number(value) & 0xffff;
  return "0x" + num.toString(16).toUpperCase().padStart(4, "0");
}

function asString(value) {
  return typeof value === "string" ? value.trim() : "";
}

function normalizeMac(value) {
  return asString(value).toUpperCase();
}

function nullableTypeCode(typeCode) {
  return typeCode === null ? "unknown" : String(typeCode);
}

function boolYaml(value) {
  return value ? "true" : "false";
}

function yamlQuote(value) {
  const escaped = String(value).replace(/\\/g, "\\\\").replace(/"/g, '\\"');
  return `"${escaped}"`;
}

function escapeHtml(value) {
  return String(value)
    .replace(/&/g, "&amp;")
    .replace(/</g, "&lt;")
    .replace(/>/g, "&gt;")
    .replace(/"/g, "&quot;")
    .replace(/'/g, "&#39;");
}
