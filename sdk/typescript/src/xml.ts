/** A small XML reader for robot descriptions: elements, attributes, text and comments. No DTDs, entities beyond the five predefined ones, namespaces or processing beyond skipping `<?...?>`. */

export interface XmlElement {
    name: string;
    /** No prototype, so an attribute called `constructor` is an ordinary attribute. */
    attrs: Record<string, string>;
    children: XmlElement[];
    text: string;
}

export const MAX_XML_BYTES = 16 << 20;
export const MAX_XML_DEPTH = 64;
export const MAX_XML_ELEMENTS = 200_000;

const ENTITIES: Record<string, string> = { '&lt;': '<', '&gt;': '>', '&amp;': '&', '&quot;': '"', '&apos;': "'" };
export class XmlError extends Error {}

function decode(s: string): string {
    return s.replace(/&(#x[0-9A-Fa-f]{1,6}|#[0-9]{1,7}|[A-Za-z]+);/g, (m, body: string) => {
        if (body[0] !== '#') {
            if (!(m in ENTITIES)) throw new XmlError(`unknown entity ${m}`);
            return ENTITIES[m];
        }
        const code = body[1] === 'x' ? parseInt(body.slice(2), 16) : parseInt(body.slice(1), 10);
        if (code === 0 || code > 0x10ffff || (code >= 0xd800 && code <= 0xdfff)) throw new XmlError(`invalid character reference ${m}`);
        return String.fromCodePoint(code);
    });
}

export function parseXml(source: string): XmlElement {
    if (source.length > MAX_XML_BYTES) throw new XmlError(`document is larger than ${MAX_XML_BYTES} characters`);
    // A DTD or a custom entity would need resolving, which this reader refuses to do. Comments and CDATA may mention them.
    if (/<!DOCTYPE|<!ENTITY/i.test(source.replace(/<!--[\s\S]*?-->|<!\[CDATA\[[\s\S]*?\]\]>/g, ''))) throw new XmlError('DOCTYPE and ENTITY declarations are not supported');
    let i = 0;
    const stack: XmlElement[] = [];
    let root: XmlElement | null = null;
    const fail = (msg: string): never => { throw new XmlError(`${msg} at offset ${i}`); };
    let elements = 0;
    const tag = /<(\/?)([A-Za-z_][\w.:-]*)((?:\s+[A-Za-z_][\w.:-]*\s*=\s*(?:"[^"<]*"|'[^'<]*'))*)\s*(\/?)>/y;
    const attr = /([A-Za-z_][\w.:-]*)\s*=\s*(?:"([^"]*)"|'([^']*)')/g;
    while (i < source.length) {
        const lt = source.indexOf('<', i);
        const text = source.slice(i, lt < 0 ? source.length : lt);
        if (text.trim() !== '') {
            if (stack.length === 0) fail('text outside the root element');
            stack[stack.length - 1].text += decode(text);
        }
        if (lt < 0) break;
        i = lt;
        if (source.startsWith('<!--', i)) {
            const end = source.indexOf('-->', i + 4);
            if (end < 0) fail('unterminated comment');
            i = end + 3;
        } else if (source.startsWith('<?', i)) {
            const end = source.indexOf('?>', i + 2);
            if (end < 0) fail('unterminated processing instruction');
            i = end + 2;
        } else if (source.startsWith('<![CDATA[', i)) {
            const end = source.indexOf(']]>', i + 9);
            if (end < 0 || stack.length === 0) fail('bad CDATA section');
            stack[stack.length - 1].text += source.slice(i + 9, end);
            i = end + 3;
        } else {
            tag.lastIndex = i;
            const m = tag.exec(source);
            if (!m) return fail('malformed tag');
            i = tag.lastIndex;
            const [, closing, name, attrText, selfClosing] = m;
            if (closing && (attrText !== '' || selfClosing)) fail(`the closing tag </${name}> must be bare`);
            if (closing) {
                const open = stack.pop();
                if (!open || open.name !== name) fail(`closing </${name}> does not match ${open ? `<${open.name}>` : 'anything'}`);
                continue;
            }
            if (++elements > MAX_XML_ELEMENTS) fail(`more than ${MAX_XML_ELEMENTS} elements`);
            const attrs: Record<string, string> = Object.create(null);
            for (const a of attrText.matchAll(attr)) {
                if (Object.hasOwn(attrs, a[1])) fail(`duplicate attribute ${a[1]}`);
                attrs[a[1]] = decode(a[2] ?? a[3]);
            }
            const el: XmlElement = { name, attrs, children: [], text: '' };
            if (stack.length) stack[stack.length - 1].children.push(el);
            else if (root) fail('more than one root element');
            else root = el;
            if (!selfClosing) {
                if (stack.length >= MAX_XML_DEPTH) fail(`nesting deeper than ${MAX_XML_DEPTH}`);
                stack.push(el);
            }
        }
    }
    if (stack.length) throw new XmlError(`<${stack[stack.length - 1].name}> is never closed`);
    if (!root) throw new XmlError('no root element');
    return root;
}
