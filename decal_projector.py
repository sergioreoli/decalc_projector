bl_info = {
    "name": "Decal Projector",
    "author": "Claude",
    "version": (1, 0, 0),
    "blender": (3, 6, 0),
    "location": "View3D > Sidebar (N) > aba Decal",
    "description": "Aplica decals em superfícies curvas ou meshes deformadas, projetando uma grade sobre o alvo",
    "category": "Object",
}

import traceback
from math import radians

import bmesh
import bpy
import gpu
from bpy.props import (
    BoolProperty,
    EnumProperty,
    FloatProperty,
    IntProperty,
    PointerProperty,
)
from bpy_extras import view3d_utils
from gpu_extras.batch import batch_for_shader
from mathutils import Matrix, Vector
from mathutils.bvhtree import BVHTree


# ----------------------------------------------------------------------------
# Configurações
# ----------------------------------------------------------------------------

class DECAL_Settings(bpy.types.PropertyGroup):
    image: PointerProperty(name="Imagem", type=bpy.types.Image)
    width: FloatProperty(
        name="Largura", default=0.5, min=0.001, soft_max=10.0,
        subtype='DISTANCE', unit='LENGTH',
    )
    keep_aspect: BoolProperty(name="Manter proporção da imagem", default=True)
    height: FloatProperty(
        name="Altura", default=0.5, min=0.001, soft_max=10.0,
        subtype='DISTANCE', unit='LENGTH',
    )
    rotation: FloatProperty(name="Rotação", default=0.0, subtype='ANGLE')

    mode: EnumProperty(
        name="Modo",
        items=[
            ('PROJECT', "Projeção reta",
             "Projeta cada vértice ao longo da normal do ponto clicado (estilo projetor de slide)"),
            ('NEAREST', "Ponto mais próximo",
             "Cola cada vértice no ponto mais próximo da superfície (bom para formas muito curvas)"),
        ],
        default='PROJECT',
    )
    resolution: IntProperty(
        name="Subdivisões", default=32, min=2, max=256,
        description="Quantidade de divisões da grade da decal no lado maior",
    )
    reach: FloatProperty(
        name="Alcance", default=1.0, min=0.05, max=10.0,
        description="Distância máxima de busca, relativa ao tamanho da decal",
    )
    offset: FloatProperty(
        name="Offset", default=0.002, min=0.0, soft_max=0.05, precision=4,
        subtype='DISTANCE', unit='LENGTH',
        description="Afastamento da superfície para evitar z-fighting",
    )
    cull: BoolProperty(
        name="Cortar fora da superfície", default=True,
        description="Remove a geometria da decal que não encontrou superfície (bordas do alvo)",
    )

    opacity: FloatProperty(name="Opacidade", default=1.0, min=0.0, max=1.0)
    roughness: FloatProperty(name="Rugosidade", default=0.5, min=0.0, max=1.0)

    only_active: BoolProperty(
        name="Só no objeto ativo", default=False,
        description="Ignora outros objetos ao posicionar a decal",
    )
    parent_to_target: BoolProperty(name="Parentear ao alvo", default=True)
    smooth: BoolProperty(name="Shade Smooth", default=True)
    use_surface_deform: BoolProperty(
        name="Seguir deformação (Surface Deform)", default=False,
        description="Adiciona um Surface Deform ligado ao alvo para a decal acompanhar animações/armature",
    )


# ----------------------------------------------------------------------------
# Utilidades
# ----------------------------------------------------------------------------

def decal_size(s):
    w = s.width
    img = s.image
    if s.keep_aspect and img and img.size[0] > 0 and img.size[1] > 0:
        h = w * img.size[1] / img.size[0]
    else:
        h = s.height
    return w, h


def decal_matrix(loc, normal, view_up, rotation):
    """Matriz mundo da decal: Z = normal, Y = 'cima' da tela projetado no plano."""
    z = normal.normalized()
    up = view_up - z * view_up.dot(z)
    if up.length < 1e-5:
        up = Vector((0.0, 1.0, 0.0)) - z * z.y
        if up.length < 1e-5:
            up = Vector((1.0, 0.0, 0.0)) - z * z.x
    y = up.normalized()
    x = y.cross(z).normalized()
    m = Matrix((x, y, z)).transposed().to_4x4()
    m.translation = loc
    return m @ Matrix.Rotation(rotation, 4, 'Z')


def get_decal_material(image, opacity, roughness):
    key = f"{image.name}|{opacity:.2f}|{roughness:.2f}"
    for m in bpy.data.materials:
        if m.get("decal_key") == key:
            return m

    mat = bpy.data.materials.new(f"Decal_{image.name}"[:50])
    mat["decal_key"] = key
    mat.use_nodes = True
    nt = mat.node_tree
    nt.nodes.clear()

    out = nt.nodes.new('ShaderNodeOutputMaterial')
    out.location = (600, 0)
    bsdf = nt.nodes.new('ShaderNodeBsdfPrincipled')
    bsdf.location = (300, 0)
    bsdf.inputs['Roughness'].default_value = roughness
    tex = nt.nodes.new('ShaderNodeTexImage')
    tex.location = (-200, 0)
    tex.image = image
    tex.extension = 'CLIP'
    tex.interpolation = 'Linear'
    mul = nt.nodes.new('ShaderNodeMath')
    mul.operation = 'MULTIPLY'
    mul.location = (50, -250)
    mul.inputs[1].default_value = opacity

    nt.links.new(tex.outputs['Color'], bsdf.inputs['Base Color'])
    nt.links.new(tex.outputs['Alpha'], mul.inputs[0])
    nt.links.new(mul.outputs[0], bsdf.inputs['Alpha'])
    nt.links.new(bsdf.outputs['BSDF'], out.inputs['Surface'])

    mat.blend_method = 'HASHED'
    mat.shadow_method = 'HASHED'
    return mat


# ----------------------------------------------------------------------------
# Projeção
# ----------------------------------------------------------------------------

class Projector:
    """Encapsula a BVH do alvo (já deformada/avaliada) e projeta pontos nela."""

    def __init__(self, target, depsgraph):
        self.target = target
        self.bvh = BVHTree.FromObject(target, depsgraph)
        self.mw = target.matrix_world.copy()
        self.inv = self.mw.inverted()
        self.inv3 = self.inv.to_3x3()
        self.nmat = self.inv.transposed().to_3x3()
        sc = self.mw.to_scale()
        self.scale = max(1e-9, (abs(sc.x) + abs(sc.y) + abs(sc.z)) / 3.0)

    def project(self, p_world, n_plane, mode, reach):
        """Retorna (posição_mundo, normal_mundo) ou None se não achou superfície."""
        if mode == 'PROJECT':
            origin = self.inv @ (p_world + n_plane * reach)
            d = self.inv3 @ (-n_plane * (2.0 * reach))
            dist = d.length
            if dist < 1e-12:
                return None
            loc, nor, _idx, _dst = self.bvh.ray_cast(origin, d / dist, dist)
        else:
            loc, nor, _idx, _dst = self.bvh.find_nearest(self.inv @ p_world, reach / self.scale)

        if loc is None:
            return None

        pos = self.mw @ loc
        n = (self.nmat @ nor).normalized()
        if n.dot(n_plane) < 0.0:
            n.negate()
        return pos, n


def build_grid(w, h, nx, ny):
    bm = bmesh.new()
    uv_layer = bm.loops.layers.uv.new("UVMap")
    verts = []
    for j in range(ny + 1):
        for i in range(nx + 1):
            verts.append(bm.verts.new(((i / nx - 0.5) * w, (j / ny - 0.5) * h, 0.0)))
    for j in range(ny):
        for i in range(nx):
            v1 = verts[j * (nx + 1) + i]
            v2 = verts[j * (nx + 1) + i + 1]
            v3 = verts[(j + 1) * (nx + 1) + i + 1]
            v4 = verts[(j + 1) * (nx + 1) + i]
            f = bm.faces.new((v1, v2, v3, v4))
            for loop in f.loops:
                co = loop.vert.co
                loop[uv_layer].uv = (co.x / w + 0.5, co.y / h + 0.5)
    bm.normal_update()
    return bm


def flatten_from_uv(bm, w, h):
    """Restaura o plano original da decal a partir das UVs (usado ao reprojetar)."""
    uv_layer = bm.loops.layers.uv.active
    if uv_layer is None:
        return False
    for f in bm.faces:
        for loop in f.loops:
            u, v = loop[uv_layer].uv
            loop.vert.co = ((u - 0.5) * w, (v - 0.5) * h, 0.0)
    return True


def project_bm(bm, mat, projector, s, w, h):
    """Projeta todos os vértices do bmesh no alvo. Retorna o nº de faces restantes."""
    mat_inv = mat.inverted()
    n_plane = (mat.to_3x3() @ Vector((0.0, 0.0, 1.0))).normalized()
    reach = max(w, h) * s.reach

    missed = []
    for v in bm.verts:
        p = mat @ v.co
        res = projector.project(p, n_plane, s.mode, reach)
        if res is None:
            if s.cull:
                missed.append(v)
            continue
        pos, nrm = res
        v.co = mat_inv @ (pos + nrm * s.offset)

    if missed:
        bmesh.ops.delete(bm, geom=missed, context='VERTS')
        loose = [v for v in bm.verts if not v.link_faces]
        if loose:
            bmesh.ops.delete(bm, geom=loose, context='VERTS')
    bm.normal_update()
    return len(bm.faces)


def outline_preview(mat, projector, s, w, h, k=12):
    """Contorno da decal já conformado na superfície, para o preview."""
    n_plane = (mat.to_3x3() @ Vector((0.0, 0.0, 1.0))).normalized()
    reach = max(w, h) * s.reach

    def snap(x, y):
        p = mat @ Vector((x, y, 0.0))
        r = projector.project(p, n_plane, s.mode, reach)
        if r is None:
            return tuple(p)
        return tuple(r[0] + r[1] * (s.offset + 0.0005))

    pts = []
    for i in range(k):
        pts.append(snap(-w / 2 + w * i / k, -h / 2))
    for i in range(k):
        pts.append(snap(w / 2, -h / 2 + h * i / k))
    for i in range(k):
        pts.append(snap(w / 2 - w * i / k, h / 2))
    for i in range(k):
        pts.append(snap(-w / 2, h / 2 - h * i / k))

    up_line = [snap(0.0, 0.0), snap(0.0, h / 2)]
    return pts, up_line


def add_surface_deform(context, decal, target):
    mod = decal.modifiers.new("DecalSD", 'SURFACE_DEFORM')
    mod.target = target
    try:
        with context.temp_override(object=decal, active_object=decal):
            bpy.ops.object.surfacedeform_bind(modifier=mod.name)
        context.view_layer.update()
        return bool(mod.is_bound)
    except Exception:
        traceback.print_exc()
        return False


def create_decal(context, projector, loc, normal, view_up, s):
    img = s.image
    w, h = decal_size(s)
    mat = decal_matrix(loc, normal, view_up, s.rotation)
    nx = s.resolution
    ny = max(2, min(256, round(nx * h / w)))

    bm = build_grid(w, h, nx, ny)
    if project_bm(bm, mat, projector, s, w, h) == 0:
        bm.free()
        return None, "A decal não encontrou superfície (aumente o Alcance)."

    mesh = bpy.data.meshes.new("Decal")
    bm.to_mesh(mesh)
    bm.free()
    mesh.materials.append(get_decal_material(img, s.opacity, s.roughness))
    if s.smooth:
        mesh.polygons.foreach_set("use_smooth", [True] * len(mesh.polygons))
    mesh.update()

    obj = bpy.data.objects.new(f"Decal_{img.name}", mesh)
    context.collection.objects.link(obj)
    obj.matrix_world = mat
    obj["is_decal"] = True
    obj["decal_target"] = projector.target.name
    obj["decal_w"] = w
    obj["decal_h"] = h

    if s.parent_to_target:
        obj.parent = projector.target
        obj.matrix_parent_inverse = projector.target.matrix_world.inverted()

    msg = ""
    if s.use_surface_deform:
        if not add_surface_deform(context, obj, projector.target):
            msg = "Surface Deform não conseguiu fazer o bind (veja o console)."

    for o in context.selected_objects:
        o.select_set(False)
    obj.select_set(True)
    context.view_layer.objects.active = obj
    return obj, msg


def cast_scene(context, origin, direction, only=None):
    """Raycast da cena ignorando decals já existentes."""
    depsgraph = context.evaluated_depsgraph_get()
    d = direction.normalized()
    for _ in range(16):
        ok, loc, nor, _idx, obj, _m = context.scene.ray_cast(depsgraph, origin, d)
        if not ok:
            return None
        obj = obj.original
        if obj.type == 'MESH' and not obj.get("is_decal") and (only is None or obj == only):
            return obj, loc, nor
        origin = loc + d * 1e-4
    return None


# ----------------------------------------------------------------------------
# Operador principal: posicionar decal com o mouse
# ----------------------------------------------------------------------------

class DECAL_OT_place(bpy.types.Operator):
    bl_idname = "decal.place"
    bl_label = "Posicionar Decal"
    bl_description = (
        "Clique na superfície para aplicar. Shift+Scroll: tamanho | Ctrl+Scroll: rotação | "
        "Botão direito/Esc: sair"
    )
    bl_options = {'REGISTER', 'UNDO'}

    @classmethod
    def poll(cls, context):
        return (
            context.area is not None
            and context.area.type == 'VIEW_3D'
            and context.mode == 'OBJECT'
        )

    # -- ciclo de vida -------------------------------------------------------
    def invoke(self, context, event):
        s = context.scene.decal_tool
        if s.image is None:
            self.report({'ERROR'}, "Selecione uma imagem para a decal.")
            return {'CANCELLED'}

        self.area = context.area
        self.region = None
        self.coord = None
        self.hit = None
        self.preview = None
        self.projectors = {}
        self.count = 0
        active = context.view_layer.objects.active
        self.only = active if (s.only_active and active and active.type == 'MESH') else None

        self._handle = bpy.types.SpaceView3D.draw_handler_add(
            self._draw, (), 'WINDOW', 'POST_VIEW'
        )
        context.window.cursor_modal_set('PAINT_CROSS')
        context.workspace.status_text_set(
            "Decal:  Clique = aplicar   |   Shift+Scroll = tamanho   |   "
            "Ctrl+Scroll = rotação   |   Botão direito / Esc = sair"
        )
        context.window_manager.modal_handler_add(self)
        return {'RUNNING_MODAL'}

    def _finish(self, context):
        if getattr(self, "_handle", None) is not None:
            bpy.types.SpaceView3D.draw_handler_remove(self._handle, 'WINDOW')
            self._handle = None
        try:
            context.window.cursor_modal_restore()
            context.workspace.status_text_set(None)
            self.area.tag_redraw()
        except Exception:
            pass
        return {'FINISHED'} if self.count else {'CANCELLED'}

    # -- desenho do preview --------------------------------------------------
    def _draw(self):
        if not self.preview:
            return
        outline, up_line = self.preview
        shader = gpu.shader.from_builtin('3D_UNIFORM_COLOR')
        gpu.state.blend_set('ALPHA')
        gpu.state.depth_test_set('NONE')
        gpu.state.line_width_set(2.0)
        shader.bind()
        shader.uniform_float("color", (1.0, 0.55, 0.1, 1.0))
        batch_for_shader(shader, 'LINE_LOOP', {"pos": outline}).draw(shader)
        shader.uniform_float("color", (0.2, 0.8, 1.0, 1.0))
        batch_for_shader(shader, 'LINES', {"pos": up_line}).draw(shader)
        gpu.state.line_width_set(1.0)
        gpu.state.blend_set('NONE')

    # -- helpers -------------------------------------------------------------
    def _region_under(self, event):
        mx, my = event.mouse_x, event.mouse_y
        # painéis (sidebar, header, toolbar...) têm prioridade sobre a viewport
        for r in self.area.regions:
            if (r.type != 'WINDOW' and r.width > 1 and r.height > 1
                    and r.x <= mx < r.x + r.width and r.y <= my < r.y + r.height):
                return None
        for r in self.area.regions:
            if (r.type == 'WINDOW'
                    and r.x <= mx < r.x + r.width and r.y <= my < r.y + r.height):
                return r
        return None

    def _projector_for(self, context, obj):
        p = self.projectors.get(obj.name)
        if p is None:
            p = Projector(obj, context.evaluated_depsgraph_get())
            self.projectors[obj.name] = p
        return p

    def _refresh(self, context):
        s = context.scene.decal_tool
        region = self.region
        rv3d = region.data
        origin = view3d_utils.region_2d_to_origin_3d(region, rv3d, self.coord)
        direction = view3d_utils.region_2d_to_vector_3d(region, rv3d, self.coord)
        hit = cast_scene(context, origin, direction, self.only)
        if hit is None:
            self.hit = None
            self.preview = None
            return
        obj, loc, nor = hit
        if nor.dot(direction) > 0.0:
            nor = -nor
        up = rv3d.view_matrix.inverted().to_3x3() @ Vector((0.0, 1.0, 0.0))
        self.hit = (obj, loc, nor, up)

        w, h = decal_size(s)
        mat = decal_matrix(loc, nor, up, s.rotation)
        self.preview = outline_preview(mat, self._projector_for(context, obj), s, w, h)

    def _place(self, context):
        if self.hit is None:
            return
        s = context.scene.decal_tool
        obj, loc, nor, up = self.hit
        decal, msg = create_decal(context, self._projector_for(context, obj), loc, nor, up, s)
        if decal is None:
            self.report({'WARNING'}, msg)
            return
        if msg:
            self.report({'WARNING'}, msg)
        self.count += 1
        try:
            bpy.ops.ed.undo_push(message="Aplicar Decal")
        except Exception:
            pass

    # -- loop modal ----------------------------------------------------------
    def modal(self, context, event):
        try:
            return self._modal(context, event)
        except ReferenceError:
            return self._finish(context)
        except Exception:
            traceback.print_exc()
            self.report({'ERROR'}, "Erro no Decal Projector (veja o console).")
            return self._finish(context)

    def _modal(self, context, event):
        s = context.scene.decal_tool

        if event.type == 'ESC' and event.value == 'PRESS':
            return self._finish(context)

        region = self._region_under(event)

        if event.type == 'MOUSEMOVE':
            if region is None:
                self.hit = None
                self.preview = None
                self.area.tag_redraw()
                return {'PASS_THROUGH'}
            self.region = region
            self.coord = (event.mouse_x - region.x, event.mouse_y - region.y)
            self._refresh(context)
            self.area.tag_redraw()
            return {'RUNNING_MODAL'}

        if region is None:
            return {'PASS_THROUGH'}

        if (event.type in {'WHEELUPMOUSE', 'WHEELDOWNMOUSE'}
                and (event.shift or event.ctrl) and not event.alt):
            up = event.type == 'WHEELUPMOUSE'
            if event.shift:
                f = 1.1 if up else 1.0 / 1.1
                s.width = max(0.001, s.width * f)
                if not s.keep_aspect:
                    s.height = max(0.001, s.height * f)
            else:
                s.rotation += radians(5.0) * (1 if up else -1)
            if self.coord is not None:
                self.region = region
                self._refresh(context)
            self.area.tag_redraw()
            return {'RUNNING_MODAL'}

        if event.type == 'LEFTMOUSE' and event.value == 'PRESS' and not event.alt:
            self.region = region
            self.coord = (event.mouse_x - region.x, event.mouse_y - region.y)
            self._refresh(context)
            self._place(context)
            self.area.tag_redraw()
            return {'RUNNING_MODAL'}

        if event.type == 'RIGHTMOUSE' and event.value == 'PRESS':
            return self._finish(context)

        return {'PASS_THROUGH'}


# ----------------------------------------------------------------------------
# Operador: reprojetar decals (útil depois de deformar/posar o alvo)
# ----------------------------------------------------------------------------

class DECAL_OT_reproject(bpy.types.Operator):
    bl_idname = "decal.reproject"
    bl_label = "Reprojetar Decals Selecionados"
    bl_description = (
        "Reprojeta as decals selecionadas no alvo (o objeto ativo, se for uma mesh comum "
        "selecionada junto; senão o alvo original). Use depois de deformar o alvo"
    )
    bl_options = {'REGISTER', 'UNDO'}

    @classmethod
    def poll(cls, context):
        return context.mode == 'OBJECT' and any(
            o.get("is_decal") for o in context.selected_objects
        )

    def execute(self, context):
        s = context.scene.decal_tool
        decals = [o for o in context.selected_objects if o.get("is_decal") and o.type == 'MESH']
        active = context.view_layer.objects.active
        override = active if (active and active.type == 'MESH' and not active.get("is_decal")) else None

        depsgraph = context.evaluated_depsgraph_get()
        projectors = {}
        done = 0

        for d in decals:
            target = override or bpy.data.objects.get(d.get("decal_target", ""))
            if target is None or target.type != 'MESH':
                self.report({'WARNING'}, f"{d.name}: alvo não encontrado.")
                continue

            had_sd = False
            for m in list(d.modifiers):
                if m.type == 'SURFACE_DEFORM':
                    d.modifiers.remove(m)
                    had_sd = True

            if target.name not in projectors:
                projectors[target.name] = Projector(target, depsgraph)

            w = d.get("decal_w")
            h = d.get("decal_h")
            if not w or not h:
                self.report({'WARNING'}, f"{d.name}: sem dados de tamanho.")
                continue

            bm = bmesh.new()
            bm.from_mesh(d.data)
            if not flatten_from_uv(bm, w, h):
                bm.free()
                self.report({'WARNING'}, f"{d.name}: sem UVs.")
                continue

            remaining = project_bm(bm, d.matrix_world.copy(), projectors[target.name], s, w, h)
            if remaining == 0:
                bm.free()
                self.report({'WARNING'}, f"{d.name}: nada encontrou superfície; mantida como estava.")
                continue

            bm.to_mesh(d.data)
            bm.free()
            d.data.update()
            d["decal_target"] = target.name

            if had_sd or s.use_surface_deform:
                if not add_surface_deform(context, d, target):
                    self.report({'WARNING'}, f"{d.name}: Surface Deform não fez o bind.")
            done += 1

        self.report({'INFO'}, f"{done} decal(s) reprojetada(s).")
        return {'FINISHED'} if done else {'CANCELLED'}


# ----------------------------------------------------------------------------
# Interface
# ----------------------------------------------------------------------------

class DECAL_PT_panel(bpy.types.Panel):
    bl_label = "Decal Projector"
    bl_idname = "DECAL_PT_panel"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "Decal"

    def draw(self, context):
        layout = self.layout
        s = context.scene.decal_tool

        layout.template_ID(s, "image", open="image.open")

        col = layout.column(align=True)
        col.prop(s, "width")
        col.prop(s, "keep_aspect")
        if not s.keep_aspect:
            col.prop(s, "height")
        col.prop(s, "rotation")

        box = layout.box()
        box.label(text="Projeção")
        box.prop(s, "mode", text="")
        box.prop(s, "resolution")
        box.prop(s, "reach")
        box.prop(s, "offset")
        box.prop(s, "cull")

        box = layout.box()
        box.label(text="Material")
        box.prop(s, "opacity")
        box.prop(s, "roughness")

        box = layout.box()
        box.label(text="Opções")
        box.prop(s, "only_active")
        box.prop(s, "parent_to_target")
        box.prop(s, "smooth")
        box.prop(s, "use_surface_deform")

        layout.separator()
        layout.operator("decal.place", icon='CURSOR')
        layout.operator("decal.reproject", icon='FILE_REFRESH')

        col = layout.column(align=True)
        col.scale_y = 0.8
        col.label(text="Shift+Scroll: tamanho")
        col.label(text="Ctrl+Scroll: rotação")
        col.label(text="Clique direito / Esc: sair")


# ----------------------------------------------------------------------------
# Registro
# ----------------------------------------------------------------------------

classes = (
    DECAL_Settings,
    DECAL_OT_place,
    DECAL_OT_reproject,
    DECAL_PT_panel,
)


def register():
    for c in classes:
        bpy.utils.register_class(c)
    bpy.types.Scene.decal_tool = PointerProperty(type=DECAL_Settings)


def unregister():
    del bpy.types.Scene.decal_tool
    for c in reversed(classes):
        bpy.utils.unregister_class(c)


if __name__ == "__main__":
    register()
