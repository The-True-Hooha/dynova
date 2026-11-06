import { assertIdOffset, decodeTransforms, type AdapterOptions, type InstanceSink, type TransformSource, type UpdateResult } from '@dynova/sdk';

/** The subset of CanvasRenderingContext2D the adapter uses; lets tests and other canvases stand in. */
export interface Canvas2DContext {
    save(): void;
    restore(): void;
    translate(x: number, y: number): void;
    rotate(angle: number): void;
    fillRect(x: number, y: number, w: number, h: number): void;
    beginPath(): void;
    arc(x: number, y: number, radius: number, start: number, end: number): void;
    fill(): void;
    fillStyle: string | CanvasGradient | CanvasPattern;
}

export interface EntityStyle {
    /** A circle is drawn with diameter `scale x` (`scale y` is ignored). */
    shape?: 'rect' | 'circle';
    color?: string;
}

export interface Canvas2DView {
    /** Screen pixels per world unit. Default 20. */
    pixelsPerUnit?: number;
    /** Screen position of the world origin; world +y points up the screen. Default (0, 0). */
    originX?: number;
    originY?: number;
}

export interface Canvas2DOptions extends AdapterOptions {
    view?: Canvas2DView;
    /** Per entity id. Default: grey rectangle. */
    style?: (id: number) => EntityStyle;
}

export interface Canvas2DAdapter {
    /**
     * Draws the latest frame as a 2D view of the x/y plane: position x/y, rotation of the body's x axis projected onto
     * the plane, scale x/y. Entities are drawn in id order (z is ignored). The caller clears the canvas.
     */
    update(): UpdateResult;
}

export function canvas2dAdapter(source: TransformSource, ctx: Canvas2DContext, options: Canvas2DOptions = {}): Canvas2DAdapter {
    assertIdOffset(options.idOffset ?? 0);
    const ppu = options.view?.pixelsPerUnit ?? 20;
    const originX = options.view?.originX ?? 0;
    const originY = options.view?.originY ?? 0;
    if (!(ppu > 0) || !Number.isFinite(ppu)) throw new RangeError(`pixelsPerUnit must be a positive finite number, got ${ppu}`);
    if (!Number.isFinite(originX) || !Number.isFinite(originY)) throw new RangeError('view origin must be finite');

    return {
        update() {
            const sink: InstanceSink = {
                show(_slot, id, x, y, _z, qx, qy, qz, qw, sx, sy) {
                    const style = options.style?.(id) ?? {};
                    // Angle of the body's x axis projected onto the x/y plane.
                    const angle = Math.atan2(2 * (qx * qy + qw * qz), 1 - 2 * (qy * qy + qz * qz));
                    const w = sx * ppu, h = sy * ppu;
                    ctx.save();
                    try {
                        ctx.translate(originX + x * ppu, originY - y * ppu);
                        ctx.rotate(-angle);
                        ctx.fillStyle = style.color ?? '#888888';
                        if (style.shape === 'circle') {
                            ctx.beginPath();
                            ctx.arc(0, 0, Math.abs(w) / 2, 0, Math.PI * 2);
                            ctx.fill();
                        } else {
                            ctx.fillRect(-w / 2, -h / 2, w, h);
                        }
                    } finally {
                        ctx.restore();
                    }
                },
                hide() {},
            };
            return decodeTransforms(source, Infinity, sink, options);
        },
    };
}
