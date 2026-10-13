# decalc_projector
Decalc Projector Addon for Blender

Como instalar: em Edit > Preferences > Add-ons > Install, escolha o arquivo e ative. O painel aparece na barra lateral (N), na aba Decal.

Como usar:

Escolha a imagem. Pode ser um PNG com transparência.
Ajuste tamanho, rotação e subdivisões.
Clique em Posicionar Decal. Um contorno que já acompanha a curvatura da superfície aparece sob o mouse.
Clique para aplicar. Você pode aplicar várias seguidas. Shift+Scroll muda o tamanho, Ctrl+Scroll gira, e botão direito ou Esc sai.

Como funciona em mesh deformada:

Ao clicar, o addon cria uma grade com UV, orientada pela normal do ponto clicado e pelo “cima” da tela.
Cada vértice da grade é projetado na mesh já avaliada, ou seja, com modifiers, shape keys e armature aplicados na pose atual.
O modo Projeção reta funciona como um projetor de slide, e o modo Ponto mais próximo cola cada vértice no ponto mais perto da superfície, o que ajuda em formas muito curvas.
O Offset afasta a decal da superfície para evitar z-fighting.
Com Cortar fora da superfície ligado, a parte da decal que passa da borda do objeto é removida.
O material é criado automaticamente (Principled com alpha da imagem, blend Hashed) e reaproveitado quando a imagem e os valores são os mesmos.

Para mesh que muda depois:

Reprojetar Decals Selecionados reprojeta a decal na pose ou forma atual do alvo. Selecione a decal, e se quiser trocar o alvo, selecione a mesh dele por último (ativa).
Seguir deformação (Surface Deform) adiciona um Surface Deform ligado ao alvo, para a decal acompanhar animação. O bind pode falhar se o alvo tiver polígonos côncavos, e nesse caso o addon avisa.

Limitações da versão atual:

Cada decal usa só a textura de cor e alpha, sem normal map ou roughness map.
Em curvaturas muito fortes, a projeção reta estica a imagem. Aumentar as subdivisões ou usar o modo “Ponto mais próximo” ajuda.projeção reta estica a imagem. Aumentar as subdivisões ou usar o modo “Ponto mais próximo” ajuda.

Se quiser, o próximo passo pode ser suporte a normal/roughness map, atlas de decals ou espelhamento (simetria).
