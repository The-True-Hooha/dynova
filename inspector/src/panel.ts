import { BODY_STATE_AWAKE, BODY_STATE_SLEEPING, BODY_STATE_STATIC, type InspectorView } from '@dynova/sdk';
import { ProfilerWindow, contactsOf, fmt, fmtBytes, fmtVec, listEntities, speed, stateCounts, stateName } from './model.js';

export interface InspectorOptions {
    view: InspectorView;
    /** Turns the runtime's debug snapshot on or off (`world.setInspector`); the panel has no other way to change the simulation. */
    setEnabled(on: boolean): void;
    /** Refresh period in milliseconds. */
    intervalMs?: number;
    /** Most entity rows drawn at once. */
    maxRows?: number;
    /** Snapshot age in ticks above which the panel warns that the simulation is not publishing. */
    staleAfterTicks?: number;
}

export interface InspectorPanel {
    update(): void;
    select(id: number | null): void;
    dispose(): void;
}

const STYLE = `
.dyn-insp{font:12px/1.4 ui-monospace,Menlo,monospace;color:#d8dee9;background:#1b1f27;border:1px solid #333b4a;border-radius:6px;padding:8px;width:340px;box-sizing:border-box;max-height:100%;overflow:auto}
.dyn-insp h4{margin:8px 0 4px;font-size:11px;letter-spacing:.06em;text-transform:uppercase;color:#8aa0c0}
.dyn-insp table{width:100%;border-collapse:collapse}
.dyn-insp td,.dyn-insp th{padding:1px 4px;text-align:left;white-space:nowrap}
.dyn-insp tr.sel{background:#2b3a55}
.dyn-insp tbody tr[data-id]{cursor:pointer}
.dyn-insp .warn{color:#ffb86c}
.dyn-insp .muted{color:#7d8799}
.dyn-insp select,.dyn-insp button{font:inherit;background:#252b36;color:inherit;border:1px solid #3a4456;border-radius:3px}
.dyn-insp .list{max-height:180px;overflow:auto}
`;

const FILTERS: [string, number | null][] = [['all', null], ['awake', BODY_STATE_AWAKE], ['sleeping', BODY_STATE_SLEEPING], ['static', BODY_STATE_STATIC]];

function el<K extends keyof HTMLElementTagNameMap>(tag: K, props: { text?: string; cls?: string; testid?: string } = {}): HTMLElementTagNameMap[K] {
    const node = document.createElement(tag);
    if (props.text !== undefined) node.textContent = props.text;
    if (props.cls) node.className = props.cls;
    if (props.testid) node.dataset.testid = props.testid;
    return node;
}

function row(table: HTMLTableSectionElement, cells: string[], testid?: string): HTMLTableRowElement {
    const tr = document.createElement('tr');
    if (testid) tr.dataset.testid = testid;
    for (const c of cells) tr.appendChild(el('td', { text: c }));
    table.appendChild(tr);
    return tr;
}

/**
 * Draws the inspector into `container`. All text goes in through `textContent`, so nothing read from the simulation can
 * inject markup. Mounting turns the debug snapshot on; `dispose()` turns it off again and removes every node it added.
 */
const mounted = new WeakSet<InspectorView>();

export function mountInspector(container: HTMLElement, options: InspectorOptions): InspectorPanel {
    const { view, setEnabled } = options;
    // Two panels would share one on/off switch: closing either would silently stop the other.
    if (mounted.has(view)) throw new Error('an inspector panel is already mounted for this view; dispose it first');
    setEnabled(true);
    mounted.add(view);
    const maxRows = options.maxRows ?? 200;
    const staleAfter = options.staleAfterTicks ?? 30;
    const profiler = new ProfilerWindow(8);

    const style = el('style', { text: STYLE });
    const root = el('div', { cls: 'dyn-insp', testid: 'inspector' });
    const status = el('div', { cls: 'muted', testid: 'status' });
    const profile = el('table');
    const profileBody = document.createElement('tbody');
    profile.appendChild(profileBody);
    const counts = el('div', { testid: 'counts' });
    const filter = el('select', { testid: 'filter' });
    for (const [name] of FILTERS) filter.appendChild(el('option', { text: name }));
    const listBox = el('div', { cls: 'list' });
    const listTable = el('table');
    const listBody = document.createElement('tbody');
    listTable.appendChild(listBody);
    listBox.appendChild(listTable);
    const listNote = el('div', { cls: 'muted', testid: 'list-note' });
    const readout = el('table', { testid: 'readout' });
    const readoutBody = document.createElement('tbody');
    readout.appendChild(readoutBody);
    const contactTable = el('table', { testid: 'contacts' });
    const contactBody = document.createElement('tbody');
    contactTable.appendChild(contactBody);

    root.append(
        el('h4', { text: 'Profiler' }), profile, status,
        el('h4', { text: 'Entities' }), counts, filter, listBox, listNote,
        el('h4', { text: 'Selected' }), readout,
        el('h4', { text: 'Contacts' }), contactTable,
    );
    container.append(style, root);

    let selected: number | null = null;
    let disposed = false;

    function render(): void {
        if (disposed) return;
        const p = view.profile();
        profiler.push(p, performance.now());
        const rates = profiler.rates();
        profileBody.replaceChildren();
        row(profileBody, ['tick', String(p.ticks)], 'p-ticks');
        row(profileBody, ['ticks/s', rates.ticksPerSecond === null ? '—' : fmt(rates.ticksPerSecond, 1)], 'p-rate');
        row(profileBody, ['step avg / last', `${rates.avgStepMs === null ? '—' : fmt(rates.avgStepMs, 2)} / ${fmt(p.stepLastMicros / 1000, 2)} ms`], 'p-step');
        row(profileBody, ['queue depth', String(p.ringPending)], 'p-queue');
        row(profileBody, ['contacts (dropped)', `${p.contacts} (${p.droppedContacts})`], 'p-contacts');
        row(profileBody, ['worker overruns', String(p.workerOverruns)], 'p-overruns');
        row(profileBody, ['memory', fmtBytes(p.memoryBytes)], 'p-memory');
        row(profileBody, ['buffer growths', String(p.growths)], 'p-growths');

        const snap = view.snapshot();
        listBody.replaceChildren();
        readoutBody.replaceChildren();
        contactBody.replaceChildren();
        if (!snap) {
            status.textContent = view.missedReads > 0 ? `waiting for a consistent frame (${view.missedReads} missed)` : 'waiting for the first frame';
            status.className = 'muted';
            counts.textContent = '';
            listNote.textContent = '';
            return;
        }
        const age = p.snapshotAgeTicks ?? 0;
        status.textContent = age > staleAfter ? `snapshot is ${age} ticks old: the simulation is paused or not publishing` : `snapshot at tick ${snap.tick}`;
        status.className = age > staleAfter ? 'warn' : 'muted';
        const c = stateCounts(snap);
        counts.textContent = `${c.awake} awake, ${c.sleeping} sleeping, ${c.static} static`;

        const wanted = FILTERS[filter.selectedIndex]?.[1] ?? null;
        const list = listEntities(snap, { state: wanted, limit: maxRows });
        for (const r of list.rows) {
            const tr = row(listBody, [String(r.id), stateName(r.state), fmtVec(r.position, 2)], `entity-${r.id}`);
            tr.dataset.id = String(r.id);
            if (r.id === selected) tr.className = 'sel';
        }
        listNote.textContent = list.total > list.rows.length ? `showing ${list.rows.length} of ${list.total}` : `${list.total} entities`;

        if (selected === null) {
            row(readoutBody, ['click an entity'], 'readout-empty');
            return;
        }
        const body = snap.body(selected);
        if (!body || body.state === 0) {
            row(readoutBody, [`entity ${selected} does not exist`], 'readout-empty');
            return;
        }
        row(readoutBody, ['id / state', `${body.id} / ${stateName(body.state)}`], 'r-state');
        row(readoutBody, ['position', fmtVec(body.position)], 'r-position');
        row(readoutBody, ['rotation', fmtVec(body.rotation)], 'r-rotation');
        row(readoutBody, ['velocity', `${fmtVec(body.velocity)} (${fmt(speed(body))} m/s)`], 'r-velocity');
        row(readoutBody, ['angular vel.', fmtVec(body.angularVelocity)], 'r-angular');
        const touching = contactsOf(snap, selected);
        for (const t of touching.slice(0, 12)) {
            row(contactBody, [`↔ ${t.other}`, fmtVec(t.point, 2), `n ${fmtVec(t.normal, 2)}`, `d ${fmt(t.depth, 3)}`], 'contact');
        }
        if (touching.length > 12) row(contactBody, [`… and ${touching.length - 12} more`]);
        if (touching.length === 0) row(contactBody, ['no contacts'], 'no-contacts');
    }

    const onClick = (e: Event) => {
        const tr = (e.target as Element).closest('tr[data-id]') as HTMLElement | null;
        if (tr) panel.select(Number(tr.dataset.id));
    };
    listBody.addEventListener('click', onClick);
    filter.addEventListener('change', render);

    const panel: InspectorPanel = {
        update: render,
        select(id) {
            selected = id;
            render();
        },
        dispose() {
            if (disposed) return;
            disposed = true;
            clearInterval(timer);
            mounted.delete(view);
            style.remove();
            root.remove();
            // Last: a disposed world makes this throw, and the panel must already be gone by then.
            setEnabled(false);
        },
    };

    render();
    const timer = setInterval(render, options.intervalMs ?? 250);
    return panel;
}
